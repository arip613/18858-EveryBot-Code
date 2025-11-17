import './global.css';
import {RootProvider} from 'fumadocs-ui/provider';
import {Inter} from 'next/font/google';
import type {ReactNode} from 'react';
import {Banner} from "fumadocs-ui/components/banner";
import Link from "next/link";
import {Provider} from "@/app/provider";
import 'katex/dist/katex.css';

const inter = Inter({
    subsets: ['latin'],
});

export const metadata = {
    icons: {
        icon: `${process.env.BASE_PATH}/favicon.ico`
    }
}

export default function Layout({children}: { children: ReactNode }) {
    return (
        <html lang="en" className={inter.className} suppressHydrationWarning>
        <body className="flex flex-col min-h-screen">
            <Provider>{children}</Provider>
        </body>
        </html>
    );
}
