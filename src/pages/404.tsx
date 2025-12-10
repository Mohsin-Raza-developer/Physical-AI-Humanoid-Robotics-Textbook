import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function Error404Header() {
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          Page Not Found
        </Heading>
        <p className="hero__subtitle">The page you requested doesn't exist.</p>
      </div>
    </header>
  );
}

export default function Error404Page(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Page Not Found | ${siteConfig.title}`}
      description="404 Page Not Found - Physical AI & Humanoid Robotics Textbook">
      <Error404Header />
      <main>
        <div className="container">
          <div className="row">
            <div className="col col--8 col--offset-2 margin-top--lg">
              <p>
                We couldn't find the page you're looking for. This could be because:
              </p>
              <ul>
                <li>The page has moved or been renamed</li>
                <li>The URL was typed incorrectly</li>
                <li>The content was removed</li>
              </ul>
              
              <div className="margin-top--lg">
                <p>Here are some helpful links to get you back on track:</p>
                <ul>
                  <li><Link to="/">Return to Homepage</Link></li>
                  <li><Link to="/docs/intro">Introduction to Physical AI & Robotics</Link></li>
                  <li><Link to="/docs/module-1-ros2/intro">Start with ROS 2</Link></li>
                  <li><Link to="/docs/module-2-gazebo-unity/intro">Explore Simulation</Link></li>
                </ul>
              </div>
              
              <div className="margin-top--lg">
                <Link
                  className="button button--primary button--lg"
                  to="/">
                  Back to Homepage
                </Link>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}