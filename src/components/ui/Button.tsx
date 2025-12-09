import React from 'react';
import Link from '@docusaurus/Link';

export type ButtonVariant = 'primary' | 'secondary' | 'accent' | 'ghost';
export type ButtonSize = 'sm' | 'md' | 'lg';

interface ButtonProps {
  variant?: ButtonVariant;
  size?: ButtonSize;
  href?: string;
  onClick?: () => void;
  children: React.ReactNode;
  className?: string;
  type?: 'button' | 'submit' | 'reset';
  disabled?: boolean;
  ariaLabel?: string;
}

const variantStyles: Record<ButtonVariant, string> = {
  primary: `
    bg-white text-primary-800
    hover:bg-gray-50 hover:shadow-lg hover:-translate-y-0.5
    focus:ring-4 focus:ring-primary-300
    dark:bg-primary-500 dark:text-white dark:hover:bg-primary-600
    shadow-md
  `,
  secondary: `
    bg-gradient-to-r from-secondary-500 to-primary-600 text-white
    hover:from-secondary-600 hover:to-primary-700 hover:shadow-lg hover:-translate-y-0.5
    focus:ring-4 focus:ring-secondary-300
    dark:from-secondary-400 dark:to-primary-500
    shadow-md
  `,
  accent: `
    bg-accent-600 text-white
    hover:bg-accent-700 hover:shadow-lg hover:-translate-y-0.5
    focus:ring-4 focus:ring-accent-300
    dark:bg-accent-500 dark:hover:bg-accent-600
    shadow-md
  `,
  ghost: `
    bg-white/10 text-white border-2 border-white/30
    hover:bg-white/20 hover:border-white/50 hover:-translate-y-0.5
    focus:ring-4 focus:ring-white/30
    dark:bg-white/5 dark:border-white/20 dark:hover:bg-white/10
    backdrop-blur-sm
  `,
};

const sizeStyles: Record<ButtonSize, string> = {
  sm: 'px-4 py-2 text-sm',
  md: 'px-6 py-3 text-base',
  lg: 'px-8 py-4 text-lg',
};

export default function Button({
  variant = 'primary',
  size = 'md',
  href,
  onClick,
  children,
  className = '',
  type = 'button',
  disabled = false,
  ariaLabel,
}: ButtonProps): JSX.Element {
  const baseStyles = `
    inline-flex items-center justify-center
    font-semibold rounded-lg
    transition-all duration-200 ease-out
    focus:outline-none focus:ring-offset-2
    disabled:opacity-50 disabled:cursor-not-allowed disabled:transform-none
    ${variantStyles[variant]}
    ${sizeStyles[size]}
    ${className}
  `.replace(/\s+/g, ' ').trim();

  if (href && !disabled) {
    return (
      <Link
        to={href}
        className={baseStyles}
        aria-label={ariaLabel}
      >
        {children}
      </Link>
    );
  }

  return (
    <button
      type={type}
      onClick={onClick}
      disabled={disabled}
      className={baseStyles}
      aria-label={ariaLabel}
    >
      {children}
    </button>
  );
}
