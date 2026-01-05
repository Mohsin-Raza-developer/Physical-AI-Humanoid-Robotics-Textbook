import React from 'react';
import { RocketLaunchIcon, BookOpenIcon } from '@heroicons/react/24/outline';
import Button from '../ui/Button';
import styles from './CTASection.module.css';

export default function CTASection(): JSX.Element {
  return (
    <section className={styles.ctaSection}>
      <div className={styles.ctaBackground}>
        <div className={styles.ctaOrb1} />
        <div className={styles.ctaOrb2} />
      </div>

      <div className="container">
        <div className={styles.ctaContent}>
          <h2 className={styles.ctaTitle}>
            Ready to Build the Future of Robotics?
          </h2>
          <p className={styles.ctaSubtitle}>
            Start your journey into Physical AI and Humanoid Robotics today.
            Master cutting-edge technologies with hands-on tutorials and real-world projects.
          </p>

          <div className={styles.ctaButtons}>
            <Button
              variant="primary"
              size="lg"
              href="/docs/intro"
              icon={<RocketLaunchIcon />}
              iconPosition="left"
              ariaLabel="Start learning now"
              className={styles.ctaPrimaryButton}
            >
              Start Learning Now
            </Button>
            <Button
              variant="ghost"
              size="lg"
              href="/docs/intro"
              icon={<BookOpenIcon />}
              iconPosition="left"
              ariaLabel="Browse all modules"
              className={styles.ctaSecondaryButton}
            >
              Browse All Modules
            </Button>
          </div>

          <div className={styles.ctaFeatures}>
            <div className={styles.ctaFeature}>
              <div className={styles.ctaFeatureIcon}>✓</div>
              <span>100% Free & Open Source</span>
            </div>
            <div className={styles.ctaFeature}>
              <div className={styles.ctaFeatureIcon}>✓</div>
              <span>Hands-on Projects</span>
            </div>
            <div className={styles.ctaFeature}>
              <div className={styles.ctaFeatureIcon}>✓</div>
              <span>Community Support</span>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
