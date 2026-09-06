import type {Metadata} from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Aurora Borealis Operator Console",
  description: "Real-time multi-sensor tracking observability dashboard",
};

export default function RootLayout({
  children,
}: Readonly<{children: React.ReactNode}>) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
