import React, { useEffect, useRef, useState } from 'react';
import { RocketLaunchIcon, CommandLineIcon, SparklesIcon } from '@heroicons/react/24/outline';
import Button from '../ui/Button';
import styles from './Hero.module.css';

export default function Hero(): JSX.Element {
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });
  const heroRef = useRef<HTMLElement>(null);

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (heroRef.current) {
        const rect = heroRef.current.getBoundingClientRect();
        setMousePosition({
          x: ((e.clientX - rect.left) / rect.width) * 100,
          y: ((e.clientY - rect.top) / rect.height) * 100,
        });
      }
    };

    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, []);

  return (
    <section className={styles.hero} ref={heroRef}>
      {/* Animated Background */}
      <div className={styles.heroBackground}>
        <div
          className={styles.heroGradientOrb}
          style={{
            left: `${mousePosition.x}%`,
            top: `${mousePosition.y}%`,
          }}
        />
        <div className={styles.heroParticles}>
          {Array.from({ length: 20 }).map((_, i) => (
            <div
              key={i}
              className={styles.particle}
              style={{
                left: `${Math.random() * 100}%`,
                top: `${Math.random() * 100}%`,
                animationDelay: `${Math.random() * 3}s`,
                animationDuration: `${3 + Math.random() * 4}s`,
              }}
            />
          ))}
        </div>
      </div>

      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <div className={styles.heroBadge}>
            <SparklesIcon className={styles.heroBadgeIcon} />
            <span>Interactive Learning Experience</span>
          </div>

          <h1 className={styles.heroTitle}>
            Master Physical AI &<br />Humanoid Robotics
          </h1>

          <p className={styles.heroSubtitle}>
            Interactive textbook for learning ROS 2, Gazebo, Isaac Sim, and Vision-Language-Action models
            in robotics development
          </p>

          <div className={styles.heroButtons}>
            <Button
              variant="primary"
              size="lg"
              href="/docs/intro"
              icon={<RocketLaunchIcon />}
              iconPosition="left"
              ariaLabel="Start learning Physical AI and Humanoid Robotics"
              className={styles.heroPrimaryButton}
            >
              Get Started
            </Button>
            <Button
              variant="secondary"
              size="lg"
              href="https://github.com/Mohsin-Raza-developer/Physical-AI-Humanoid-Robotics-Textbook"
              icon={<CommandLineIcon />}
              iconPosition="left"
              ariaLabel="View project source code on GitHub"
              className={styles.heroSecondaryButton}
            >
              View on GitHub
            </Button>
          </div>
        </div>
      </div>
    </section>
  );
}
