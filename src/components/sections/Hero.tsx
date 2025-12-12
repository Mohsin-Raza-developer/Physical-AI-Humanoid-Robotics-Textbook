import React from 'react';
import { RocketLaunchIcon, CommandLineIcon } from '@heroicons/react/24/outline';
import Button from '../ui/Button';
import styles from './Hero.module.css';

export default function Hero(): JSX.Element {
  return (
    <section className={styles.hero}>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>
            Master Physical AI & Humanoid Robotics
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
