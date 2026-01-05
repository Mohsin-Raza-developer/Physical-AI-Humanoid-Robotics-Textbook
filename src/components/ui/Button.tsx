import React, { useState, useRef, useEffect } from 'react';
import Link from '@docusaurus/Link';
import styles from './Button.module.css';

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
  icon?: React.ReactNode;
  iconPosition?: 'left' | 'right';
}

interface Ripple {
  x: number;
  y: number;
  size: number;
  id: number;
}

const variantStyles: Record<ButtonVariant, string> = {
  primary: `
    bg-gradient-to-r from-primary-600 to-primary-800 text-white
    hover:from-primary-700 hover:to-primary-900 hover:shadow-xl hover:-translate-y-0.5
    focus:ring-4 focus:ring-primary-300
    dark:bg-gradient-to-r dark:from-primary-500 dark:to-primary-700 dark:hover:from-primary-600 dark:hover:to-primary-800
    shadow-lg
  `,
  secondary: `
    bg-gradient-to-r from-secondary-500 to-primary-600 text-white
    hover:from-secondary-600 hover:to-primary-700 hover:shadow-xl hover:-translate-y-0.5
    focus:ring-4 focus:ring-secondary-300
    dark:bg-white/10 dark:backdrop-blur-md dark:border dark:border-white/30
    dark:hover:bg-white/20 dark:hover:border-white/40
    shadow-lg
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
  icon,
  iconPosition = 'left',
}: ButtonProps): JSX.Element {
  const [ripples, setRipples] = useState<Ripple[]>([]);
  const [isHovered, setIsHovered] = useState(false);
  const buttonRef = useRef<HTMLElement>(null);

  const createRipple = (event: React.MouseEvent<HTMLElement>) => {
    if (disabled) return;

    const button = event.currentTarget;
    const rect = button.getBoundingClientRect();
    const size = Math.max(rect.width, rect.height);
    const x = event.clientX - rect.left - size / 2;
    const y = event.clientY - rect.top - size / 2;

    const newRipple: Ripple = {
      x,
      y,
      size,
      id: Date.now(),
    };

    setRipples((prev) => [...prev, newRipple]);

    setTimeout(() => {
      setRipples((prev) => prev.filter((r) => r.id !== newRipple.id));
    }, 600);
  };

  const handleClick = (event: React.MouseEvent<HTMLElement>) => {
    createRipple(event);
    if (onClick) onClick();
  };

  const baseStyles = `
    ${styles.button}
    ${styles[`button--${variant}`]}
    ${styles[`button--${size}`]}
    ${className}
  `.replace(/\s+/g, ' ').trim();

  const content = (
    <>
      <span className={styles.buttonContent}>
        {icon && iconPosition === 'left' && (
          <span className={`${styles.buttonIcon} ${isHovered ? styles['buttonIcon--animated'] : ''}`}>
            {icon}
          </span>
        )}
        <span className={styles.buttonText}>{children}</span>
        {icon && iconPosition === 'right' && (
          <span className={`${styles.buttonIcon} ${isHovered ? styles['buttonIcon--animated'] : ''}`}>
            {icon}
          </span>
        )}
      </span>
      <div className={styles.buttonRipples}>
        {ripples.map((ripple) => (
          <span
            key={ripple.id}
            className={styles.buttonRipple}
            style={{
              left: ripple.x,
              top: ripple.y,
              width: ripple.size,
              height: ripple.size,
            }}
          />
        ))}
      </div>
    </>
  );

  if (href && !disabled) {
    return (
      <Link
        to={href}
        className={baseStyles}
        aria-label={ariaLabel}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
        onClick={handleClick as any}
        ref={buttonRef as any}
      >
        {content}
      </Link>
    );
  }

  return (
    <button
      type={type}
      onClick={handleClick}
      disabled={disabled}
      className={baseStyles}
      aria-label={ariaLabel}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      ref={buttonRef as any}
    >
      {content}
    </button>
  );
}
