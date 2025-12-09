import React from 'react';
import Link from '@docusaurus/Link';
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
            <Link
              className={styles.heroCta}
              to="/docs/intro">
              Get Started
            </Link>
            <Link
              className={styles.heroCtaSecondary}
              to="https://github.com/Mohsin-Raza-developer/Physical-AI-Humanoid-Robotics-Textbook">
              View on GitHub
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}
