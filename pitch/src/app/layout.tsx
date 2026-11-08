import type { Metadata, Viewport } from 'next';
import '@fontsource/archivo-black/400.css';
import '@fontsource-variable/jetbrains-mono';
import 'lenis/dist/lenis.css';
import './globals.css';

export const metadata: Metadata = {
  title: 'NAVLESS: Zero GPS. Pure Autonomy.',
  description: 'skybeta-19, autonomous indoor UAV for DroneHacks. Jetson Orin Nano, ROS 2 Humble, cuVSLAM 31 Hz stereo odometry, phone-LiDAR map, VL53L1X ToF. No GPS.',
  openGraph: {
    title: 'NAVLESS: Zero GPS. Pure Autonomy.',
    description: 'Stereo cameras, a phone-LiDAR map and IR range sensors, with a live A* + EGO replanner you can break.',
    type: 'website',
  },
};

export const viewport: Viewport = {
  themeColor: '#0d0807',
  colorScheme: 'dark',
  width: 'device-width',
  initialScale: 1,
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
