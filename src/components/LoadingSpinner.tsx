import React from 'react';
import clsx from 'clsx';
import type {Props} from '@theme/LoadingSpinner';

import styles from './styles.module.css';

export default function LoadingSpinner({className}: Props): JSX.Element {
  return (
    <div className={clsx('loading-roller', styles.spinner, className)}>
      {[...Array(8)].map((_, i) => (
        <div
          key={i}
          className={clsx(styles.rect, `loading-roller-rect-${i + 1}`)}
          style={{animationDelay: `${i * 0.1}s`}}
        />
      ))}
    </div>
  );
}