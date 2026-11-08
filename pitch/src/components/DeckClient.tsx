'use client';

import dynamic from 'next/dynamic';

/** Everything WebGL is client-only. The server renders just this black shell, so first paint is instant. */
const Deck = dynamic(() => import('./Deck'), {
  ssr: false,
  loading: () => <div className="fixed inset-0 bg-void" />,
});

export default function DeckClient() {
  return <Deck />;
}
