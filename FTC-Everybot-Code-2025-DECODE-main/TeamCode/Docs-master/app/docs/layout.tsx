import {DocsLayout} from 'fumadocs-ui/layouts/docs';
import type {ReactNode} from 'react';
import {baseOptions} from '@/app/layout.config';
import {source} from '@/lib/source';
import Discord from "@/app/Discord";
import {GithubInfo} from "fumadocs-ui/components/github-info";

export default function Layout({children}: { children: ReactNode }) {
    return (
        <DocsLayout tree={source.pageTree} {...baseOptions} links={[{
            type: 'custom',
            children: (
                <GithubInfo owner="Pedro-Pathing" repo="PedroPathing" className="lg:-mx-2"/>
            )
        }, ...baseOptions.links!]}>
            {children}
        </DocsLayout>
    );
}
