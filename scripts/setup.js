#!/usr/bin/env node
// One-command Navless setup: registers this agent with Claude Code and teaches it the protocol.
// Usage: npm run setup

import { createInterface } from "node:readline/promises";
import { execFileSync } from "node:child_process";
import { existsSync, readFileSync, appendFileSync } from "node:fs";
import { resolve, dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const serverPath = join(root, "src", "server.js");
const agentsPath = join(root, "AGENTS.md");
const DEFAULT_CHANNEL = "C0CDARV3UQG"; // #team-navless

// Line queue so prompts work both in a terminal and with piped input.
const rl = createInterface({ input: process.stdin, terminal: false });
const lines = [];
const waiters = [];
rl.on("line", (l) => (waiters.length ? waiters.shift()(l) : lines.push(l)));
rl.on("close", () => waiters.splice(0).forEach((w) => w("")));
const ask = async (q, def) => {
  process.stdout.write(def ? `${q} [${def}]: ` : `${q}: `);
  const answer = lines.length ? lines.shift() : await new Promise((r) => waiters.push(r));
  return answer.trim() || def || "";
};

console.log("\nNavless setup. Your agent will join the shared Slack channel.\n");
const token = await ask("Bot token (xoxb-..., from Raaga by DM)");
if (!token.startsWith("xoxb-")) {
  console.error("That doesn't look like a bot token (should start with xoxb-).");
  process.exit(1);
}
const channel = await ask("Channel ID", DEFAULT_CHANNEL);
const name = (await ask("Agent name, how others @mention it (e.g. lidar, meshgen)")).replace(/^@/, "");
if (!/^[a-z0-9][a-z0-9-]*$/i.test(name)) {
  console.error("Use letters, numbers and dashes only.");
  process.exit(1);
}
const owner = await ask("Your name");
const emoji = await ask("Avatar emoji", ":robot_face:");
const server = name === "agent" ? "navless" : `navless-${name}`;

const args = [
  "mcp", "add", server, "--scope", "user",
  "-e", `SLACK_BOT_TOKEN=${token}`,
  "-e", `SLACK_CHANNEL_ID=${channel}`,
  "-e", `AGENT_NAME=${name}`,
  "-e", `AGENT_OWNER=${owner}`,
  "-e", `AGENT_EMOJI=${emoji}`,
  "--", "node", serverPath,
];

try {
  execFileSync("claude", args, { stdio: "inherit" });
  console.log(`\nRegistered "${server}" with Claude Code.`);
} catch {
  console.log("\nCouldn't run `claude` (is Claude Code installed?). Use this config in your agent's mcp.json instead:\n");
  console.log(JSON.stringify({ mcpServers: { [server]: { command: "node", args: [serverPath], env: {
    SLACK_BOT_TOKEN: "<your token>", SLACK_CHANNEL_ID: channel, AGENT_NAME: name, AGENT_OWNER: owner, AGENT_EMOJI: emoji,
  } } } }, null, 2));
}

const project = await ask("Path to the project you'll work in (adds the agent rules to its CLAUDE.md, blank to skip)");
if (project) {
  const claudeMd = join(resolve(project.replace(/^~/, process.env.HOME || "~")), "CLAUDE.md");
  const line = `@${agentsPath}`;
  const current = existsSync(claudeMd) ? readFileSync(claudeMd, "utf8") : "";
  if (current.includes(line)) {
    console.log("CLAUDE.md already has the Navless rules.");
  } else {
    appendFileSync(claudeMd, `${current && !current.endsWith("\n") ? "\n" : ""}\n# Team agent channel\n${line}\n`);
    console.log(`Added Navless rules to ${claudeMd}`);
  }
}

rl.close();
console.log(`\nDone. Start Claude Code and say: "read the navless channel and post a hello"\n`);
