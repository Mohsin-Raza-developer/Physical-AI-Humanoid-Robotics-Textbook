import React, { useEffect, useRef, useState } from 'react';
import { AcademicCapIcon, BookOpenIcon, RocketLaunchIcon, UsersIcon } from '@heroicons/react/24/outline';
import styles from './StatsCounter.module.css';

interface StatItemProps {
  icon: React.ReactNode;
  value: number;
  label: string;
  suffix?: string;
}

function StatItem({ icon, value, label, suffix = '' }: StatItemProps): JSX.Element {
  const [count, setCount] = useState(0);
  const [isVisible, setIsVisible] = useState(false);
  const statRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting && !isVisible) {
          setIsVisible(true);
        }
      },
      { threshold: 0.5 }
    );

    if (statRef.current) {
      observer.observe(statRef.current);
    }

    return () => observer.disconnect();
  }, [isVisible]);

  useEffect(() => {
    if (!isVisible) return;

    const duration = 2000;
    const steps = 60;
    const increment = value / steps;
    let current = 0;

    const timer = setInterval(() => {
      current += increment;
      if (current >= value) {
        setCount(value);
        clearInterval(timer);
      } else {
        setCount(Math.floor(current));
      }
    }, duration / steps);

    return () => clearInterval(timer);
  }, [isVisible, value]);

  return (
    <div ref={statRef} className={styles.statItem}>
      <div className={styles.statIcon}>{icon}</div>
      <div className={styles.statValue}>
        {count}{suffix}
      </div>
      <div className={styles.statLabel}>{label}</div>
    </div>
  );
}

export default function StatsCounter(): JSX.Element {
  const stats = [
    {
      icon: <BookOpenIcon />,
      value: 4,
      label: 'Core Modules',
      suffix: '+',
    },
    {
      icon: <AcademicCapIcon />,
      value: 50,
      label: 'Learning Topics',
      suffix: '+',
    },
    {
      icon: <RocketLaunchIcon />,
      value: 100,
      label: 'Code Examples',
      suffix: '+',
    },
    {
      icon: <UsersIcon />,
      value: 1000,
      label: 'Active Learners',
      suffix: '+',
    },
  ];

  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className={styles.statsGrid}>
          {stats.map((stat, idx) => (
            <StatItem
              key={idx}
              icon={stat.icon}
              value={stat.value}
              label={stat.label}
              suffix={stat.suffix}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
