import React from 'react';
import { useThemeConfig } from '@docusaurus/theme-common';
import {
    splitNavbarItems,
    useNavbarMobileSidebar,
} from '@docusaurus/theme-common/internal';
import NavbarItem from '@theme/NavbarItem';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import SearchBar from '@theme/SearchBar';
import NavbarMobileSidebarToggle from '@theme/Navbar/MobileSidebar/Toggle';
import NavbarLogo from '@theme/Navbar/Logo';
import NavbarSearch from '@theme/Navbar/Search';
import UserProfileNavbar from '@site/src/components/auth/UserProfileNavbar';

import styles from './styles.module.css';

function useNavbarItems() {
    // Return items registered in themeConfig
    return useThemeConfig().navbar.items;
}

function NavbarItems({ items }: { items: any[] }) {
    return (
        <>
            {items.map((item, i) => (
                <NavbarItem {...item} key={i} />
            ))}
        </>
    );
}

function NavbarContentLayout({ left, right }: { left: React.ReactNode; right: React.ReactNode }) {
    return (
        <div className="navbar__inner">
            <div className="navbar__items">{left}</div>
            <div className="navbar__items navbar__items--right">{right}</div>
        </div>
    );
}

export default function NavbarContent(): JSX.Element {
    const mobileSidebar = useNavbarMobileSidebar();
    const items = useNavbarItems();
    const [leftItems, rightItems] = splitNavbarItems(items);
    const searchBarItem = items.find((item) => item.type === 'search');

    return (
        <NavbarContentLayout
            left={
                // TODO stop hardcoding items?
                <>
                    {!mobileSidebar.disabled && <NavbarMobileSidebarToggle />}
                    <NavbarLogo />
                    <NavbarItems items={leftItems} />
                </>
            }
            right={
                // TODO stop hardcoding items?
                <>
                    <NavbarItems items={rightItems} />
                    {/* Add UserProfileNavbar before color mode toggle */}
                    <UserProfileNavbar />
                    <NavbarColorModeToggle className={styles.colorModeToggle} />
                    {!searchBarItem && (
                        <NavbarSearch>
                            <SearchBar />
                        </NavbarSearch>
                    )}
                </>
            }
        />
    );
}
