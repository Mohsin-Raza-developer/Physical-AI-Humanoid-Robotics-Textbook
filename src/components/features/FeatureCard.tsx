import React from 'react';
import Link from '@docusaurus/Link';
import styles from './FeatureCard.module.css';

interface FeatureCardProps {
  icon: React.ReactNode;
  title: string;
  description: string;
  link: string;
}

export default function FeatureCard({ icon, title, description, link }: FeatureCardProps): JSX.Element {
  return (
    <article className={styles.featureCard}>
      <Link
        to={link}
        className={styles.featureCardLink}
        tabIndex={0}
        aria-label={`Learn more about ${title}`}
      >
        <div className={styles.featureCardIcon}>
          {icon}
        </div>
        <h3 className={styles.featureCardTitle}>{title}</h3>
        <p className={styles.featureCardDescription}>{description}</p>
      </Link>
    </article>
  );
}
