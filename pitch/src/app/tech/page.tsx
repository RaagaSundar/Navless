import type { Metadata } from 'next';
import TechPage from '@/components/tech/TechPage';

export const metadata: Metadata = {
  title: 'NAVLESS: technical companion',
  description: 'The system diagram, the evidence, what generalizes, what breaks it, every number and its source. For the other laptops.',
};

export default function Page() {
  return <TechPage />;
}
