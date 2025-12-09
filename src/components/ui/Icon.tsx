import React from 'react';
import {
  CpuChipIcon,
  RocketLaunchIcon,
  BoltIcon,
  AcademicCapIcon,
} from '@heroicons/react/24/outline';

export type IconName = 'cpu' | 'rocket' | 'bolt' | 'academic';
export type IconSize = 'sm' | 'md' | 'lg' | 'xl';

interface IconProps {
  name: IconName;
  size?: IconSize;
  className?: string;
  ariaLabel?: string;
  ariaHidden?: boolean;
}

const iconComponents: Record<IconName, React.ComponentType<React.ComponentProps<'svg'>>> = {
  cpu: CpuChipIcon,
  rocket: RocketLaunchIcon,
  bolt: BoltIcon,
  academic: AcademicCapIcon,
};

const sizeClasses: Record<IconSize, string> = {
  sm: 'w-4 h-4',
  md: 'w-6 h-6',
  lg: 'w-10 h-10',
  xl: 'w-14 h-14',
};

export default function Icon({
  name,
  size = 'md',
  className = '',
  ariaLabel,
  ariaHidden = false,
}: IconProps): JSX.Element {
  const IconComponent = iconComponents[name];

  if (!IconComponent) {
    console.warn(`Icon "${name}" not found. Available icons: ${Object.keys(iconComponents).join(', ')}`);
    return <></>;
  }

  return (
    <IconComponent
      className={`${sizeClasses[size]} ${className}`.trim()}
      aria-label={ariaLabel}
      aria-hidden={ariaHidden}
      role={ariaHidden ? undefined : 'img'}
    />
  );
}

// Named exports for direct icon access
export {
  CpuChipIcon,
  RocketLaunchIcon,
  BoltIcon,
  AcademicCapIcon,
};
