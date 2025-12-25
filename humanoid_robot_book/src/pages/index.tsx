import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import Translate from '@docusaurus/Translate';

import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroContent}>
        <div className={styles.heroText}>
          <Heading as="h1" className={styles.heroTitle}>
            <Translate id="homepage.hero.title">
              Physical AI & Humanoid Robotics
            </Translate>
          </Heading>
          <p className={styles.heroSubtitle}>
            <Translate id="homepage.hero.subtitle">
              The Embodied Intelligence Course
            </Translate>
          </p>
          <p className={styles.heroDescription}>
            <Translate id="homepage.hero.description">
              Bridging the gap between the digital brain and the physical body
            </Translate>
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--lg', styles.ctaButton)}
              to="/docs/introduction">
              <Translate id="homepage.cta.getStarted">
                Get Started
              </Translate>
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
            <h3>
              <Translate id="homepage.features.focus.title">
                FOCUS
              </Translate>
            </h3>
            <p>
              <Translate id="homepage.features.focus.description">
                AI Systems in the Physical World
              </Translate>
            </p>
          </div>
          <div className={styles.compactCard}>
            <h3>
              <Translate id="homepage.features.goal.title">
                GOAL
              </Translate>
            </h3>
            <p>
              <Translate id="homepage.features.goal.description">
                Master the integration of artificial intelligence with embodied robotics
              </Translate>
            </p>
          </div>
          <div className={styles.compactCard}>
            <h3>
              <Translate id="homepage.features.approach.title">
                APPROACH
              </Translate>
            </h3>
            <p>
              <Translate id="homepage.features.approach.description">
                Theory, practical implementation, and real-world applications
              </Translate>
            </p>
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
