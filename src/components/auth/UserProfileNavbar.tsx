import React, { useState, useEffect, useRef } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from './AuthProvider';
import './UserProfileNavbar.css';

export default function UserProfileNavbar(): JSX.Element {
    const { siteConfig } = useDocusaurusContext();
    const { state, logout } = useAuth();
    const { user } = state;
    const [isDropdownOpen, setIsDropdownOpen] = useState(false);
    const dropdownRef = useRef<HTMLDivElement>(null);

    // Close dropdown when clicking outside
    useEffect(() => {
        function handleClickOutside(event: MouseEvent) {
            if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
                setIsDropdownOpen(false);
            }
        }

        document.addEventListener('mousedown', handleClickOutside);
        return () => document.removeEventListener('mousedown', handleClickOutside);
    }, []);

    const handleLogout = async () => {
        await logout();
        setIsDropdownOpen(false);
        window.location.href = siteConfig.baseUrl;
    };

    // If user is not logged in, show Sign In and Sign Up buttons
    if (!user) {
        return (
            <div className="auth-buttons-container">
                <Link to="/auth/login" className="signin-button">
                    Sign In
                </Link>
                <Link
                    to="/auth/register"
                    className="button button--primary button--lg signup-button">
                    Sign Up
                </Link>
            </div>
        );
    }

    // If user is logged in, show profile menu
    const firstName = user.firstName || user.email?.split('@')[0] || 'User';
    const lastName = user.lastName || '';
    const fullName = `${firstName} ${lastName}`.trim();
    const initial = firstName.charAt(0).toUpperCase();

    return (
        <div className="navbar-user-menu" ref={dropdownRef}>
            <button
                className="user-menu-trigger"
                onClick={() => setIsDropdownOpen(!isDropdownOpen)}
                aria-expanded={isDropdownOpen}
                aria-haspopup="true"
                aria-label="User menu">
                <div className="user-avatar">{initial}</div>
            </button>

            {isDropdownOpen && (
                <div className="dropdown-menu">
                    <div className="dropdown-header">
                        <div className="user-info">
                            <div className="user-avatar-large">{initial}</div>
                            <div className="user-details">
                                <div className="user-full-name">{fullName}</div>
                                <div className="user-email">{user.email}</div>
                            </div>
                        </div>
                    </div>

                    <div className="dropdown-divider"></div>

                    <Link to="/auth/profile" className="dropdown-item" onClick={() => setIsDropdownOpen(false)}>
                        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                            <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
                            <circle cx="12" cy="7" r="4" />
                        </svg>
                        Profile
                    </Link>

                    <div className="dropdown-divider"></div>

                    <button onClick={handleLogout} className="dropdown-item logout-button">
                        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                            <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4" />
                            <polyline points="16 17 21 12 16 7" />
                            <line x1="21" y1="12" x2="9" y2="12" />
                        </svg>
                        Sign Out
                    </button>
                </div>
            )}
        </div>
    );
}
