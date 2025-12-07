import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroContent}>
        <div className={styles.heroText}>
          <Heading as="h1" className={styles.heroTitle}>
            Physical AI & Humanoid Robotics
          </Heading>
          <p className={styles.heroSubtitle}>The Embodied Intelligence Course</p>
          <p className={styles.heroDescription}>
            Bridging the gap between the digital brain and the physical body
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--lg', styles.ctaButton)}
              to="/docs/introduction">
              Get Started
            </Link>
          </div>
        </div>
        <div className={styles.heroImageContainer}>
          <img
            src="https://community.firstinspires.org/hubfs/_6528a50b-2d80-4673-a4c0-c7b1e4124cec.jpg"
            alt="FIRST Robotics Competition Robot"
            className={styles.robotImage}
          />
        </div>
      </div>
    </header>
  );
}

function CompactFeatures() {
  return (
    <section className={styles.compactFeatures}>
      <div className="container">
        <div className={styles.compactGrid}>
          <div className={styles.compactCard}>
            <h3>FOCUS</h3>
            <p>AI Systems in the Physical World</p>
          </div>
          <div className={styles.compactCard}>
            <h3>GOAL</h3>
            <p>Master the integration of artificial intelligence with embodied robotics</p>
          </div>
          <div className={styles.compactCard}>
            <h3>APPROACH</h3>
            <p>Theory, practical implementation, and real-world applications</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn about Physical AI and Humanoid Robotics - bridging the gap between digital intelligence and physical embodiment">
      <HomepageHeader />
      <main>
        <CompactFeatures />
      </main>
    </Layout>
  );
}
