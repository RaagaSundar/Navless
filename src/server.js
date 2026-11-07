#!/usr/bin/env node
// Navless: a shared Slack channel where every teammate's agent can post, read, and hand off work.
// Runs as an MCP server over stdio. One instance per agent; each has its own AGENT_NAME.

import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { WebClient } from "@slack/web-api";
import { z } from "zod";

const SLACK_BOT_TOKEN = process.env.SLACK_BOT_TOKEN;
const SLACK_CHANNEL_ID = process.env.SLACK_CHANNEL_ID;
const AGENT_NAME = (process.env.AGENT_NAME || "agent").trim();
const AGENT_OWNER = (process.env.AGENT_OWNER || "").trim();
const AGENT_EMOJI = process.env.AGENT_EMOJI || ":robot_face:";
const POLL_SECONDS = Number(process.env.NAVLESS_POLL_SECONDS || 4);

if (!SLACK_BOT_TOKEN || !SLACK_CHANNEL_ID) {
  console.error("navless: set SLACK_BOT_TOKEN and SLACK_CHANNEL_ID (see README.md)");
  process.exit(1);
}

const slack = new WebClient(SLACK_BOT_TOKEN, {
  ...(process.env.SLACK_API_URL ? { slackApiUrl: process.env.SLACK_API_URL } : {}),
});

const displayName = AGENT_OWNER ? `${AGENT_NAME} (${AGENT_OWNER})` : AGENT_NAME;
const userNames = new Map();

async function nameForUser(userId) {
  if (!userId) return "unknown";
  if (userNames.has(userId)) return userNames.get(userId);
  let name = userId;
  try {
    const res = await slack.users.info({ user: userId });
    name = res.user?.profile?.display_name || res.user?.real_name || res.user?.name || userId;
  } catch {
    // users:read missing or lookup failed; fall back to the raw id
  }
  userNames.set(userId, name);
  return name;
}

// Turn a raw Slack message into something an agent can reason about.
async function describe(msg) {
  const isAgent = Boolean(msg.bot_id && msg.username);
  const author = isAgent ? msg.username : await nameForUser(msg.user);
  return {
    ts: msg.ts,
    thread_ts: msg.thread_ts && msg.thread_ts !== msg.ts ? msg.thread_ts : undefined,
    author,
    kind: isAgent ? "agent" : "human",
    from_me: isAgent && msg.username === displayName,
    mentions_me: mentionsMe(msg.text || ""),
    replies: msg.reply_count || undefined,
    text: msg.text || "",
  };
}

function mentionsMe(text) {
  const t = text.toLowerCase();
  return t.includes(`@${AGENT_NAME.toLowerCase()}`) || t.includes("@all-agents");
}

async function fetchMessages({ since, thread_ts, limit }) {
  const res = thread_ts
    ? await slack.conversations.replies({
        channel: SLACK_CHANNEL_ID,
        ts: thread_ts,
        limit,
        ...(since ? { oldest: since } : {}),
      })
    : await slack.conversations.history({
        channel: SLACK_CHANNEL_ID,
        limit,
        ...(since ? { oldest: since } : {}),
      });
  const msgs = (res.messages || []).filter((m) => !m.subtype || m.subtype === "bot_message");
  // history comes newest first; replies oldest first. Normalize to oldest first.
  msgs.sort((a, b) => Number(a.ts) - Number(b.ts));
  const out = [];
  for (const m of msgs) {
    if (since && m.ts === since) continue; // oldest is inclusive in some cases
    out.push(await describe(m));
  }
  return out;
}

function format(messages) {
  if (!messages.length) return "No messages.";
  return messages
    .map((m) => {
      const tags = [m.kind, m.from_me ? "you" : null, m.mentions_me ? "MENTIONS YOU" : null]
        .filter(Boolean)
        .join(", ");
      const thread = m.thread_ts ? ` thread=${m.thread_ts}` : "";
      const replies = m.replies ? ` (${m.replies} replies)` : "";
      return `[ts=${m.ts}${thread}] ${m.author} <${tags}>${replies}: ${m.text}`;
    })
    .join("\n");
}

const text = (s) => ({ content: [{ type: "text", text: s }] });

const server = new McpServer({ name: "navless", version: "0.1.0" });

server.registerTool(
  "slack_post",
  {
    title: "Post to the team agent channel",
    description:
      `Post a message to the shared Navless Slack channel as "${displayName}". ` +
      "Use it to announce what you're starting, report results, ask another agent for help " +
      "(address them as @their-agent-name, or @all-agents), or reply in a thread by passing thread_ts.",
    inputSchema: {
      text: z.string().min(1).describe("Message body. Slack markdown (mrkdwn) is supported."),
      thread_ts: z.string().optional().describe("ts of the message to reply to, to keep a conversation threaded."),
    },
  },
  async ({ text: body, thread_ts }) => {
    const res = await slack.chat.postMessage({
      channel: SLACK_CHANNEL_ID,
      text: body,
      username: displayName,
      icon_emoji: AGENT_EMOJI,
      ...(thread_ts ? { thread_ts } : {}),
      unfurl_links: false,
    });
    return text(`Posted as ${displayName}. ts=${res.ts}${thread_ts ? ` (in thread ${thread_ts})` : ""}`);
  }
);

server.registerTool(
  "slack_read",
  {
    title: "Read the team agent channel",
    description:
      "Read recent messages from the shared channel (agents and humans), oldest first. " +
      "Pass `since` (a ts from a previous read) to get only newer messages, or `thread_ts` to read one thread.",
    inputSchema: {
      since: z.string().optional().describe("Only return messages newer than this ts."),
      thread_ts: z.string().optional().describe("Read this thread instead of the main channel."),
      limit: z.number().int().min(1).max(100).default(25),
    },
  },
  async ({ since, thread_ts, limit }) => {
    const msgs = await fetchMessages({ since, thread_ts, limit });
    const latest = msgs.length ? msgs[msgs.length - 1].ts : since || "none";
    return text(`${format(msgs)}\n\nlatest_ts=${latest} (pass as \`since\` next time)`);
  }
);

server.registerTool(
  "slack_wait",
  {
    title: "Wait for a reply",
    description:
      "Block until a new message arrives in the channel (or thread), then return it. " +
      "Set only_mentions=true to wait specifically for a message addressed to you. " +
      "Use this after asking another agent or a human something.",
    inputSchema: {
      since: z.string().describe("ts to wait after, usually the ts of the message you just posted."),
      thread_ts: z.string().optional().describe("Watch a single thread instead of the whole channel."),
      only_mentions: z.boolean().default(false),
      timeout_seconds: z.number().int().min(5).max(600).default(120),
    },
  },
  async ({ since, thread_ts, only_mentions, timeout_seconds }) => {
    const deadline = Date.now() + timeout_seconds * 1000;
    let cursor = since;
    while (Date.now() < deadline) {
      let msgs = await fetchMessages({ since: cursor, thread_ts, limit: 50 });
      // Watching the main channel? Also catch thread replies to the message we're waiting on.
      if (!thread_ts) {
        const replies = await fetchMessages({ since: cursor, thread_ts: since, limit: 50 }).catch(() => []);
        const seen = new Set(msgs.map((m) => m.ts));
        msgs = [...msgs, ...replies.filter((m) => !seen.has(m.ts) && m.ts !== since)];
        msgs.sort((a, b) => Number(a.ts) - Number(b.ts));
      }
      msgs = msgs.filter((m) => !m.from_me);
      if (msgs.length) cursor = msgs[msgs.length - 1].ts;
      const hits = only_mentions ? msgs.filter((m) => m.mentions_me) : msgs;
      if (hits.length) return text(`${format(hits)}\n\nlatest_ts=${cursor}`);
      await new Promise((r) => setTimeout(r, POLL_SECONDS * 1000));
    }
    return text(`No new ${only_mentions ? "mentions" : "messages"} after ${timeout_seconds}s. latest_ts=${cursor}`);
  }
);

server.registerTool(
  "slack_whoami",
  {
    title: "Navless identity",
    description: "Return this agent's name in the channel and how others should address it.",
    inputSchema: {},
  },
  async () =>
    text(`You post as "${displayName}". Other agents reach you with @${AGENT_NAME}; everyone with @all-agents.`)
);

await server.connect(new StdioServerTransport());
console.error(`navless: connected as ${displayName}`);
