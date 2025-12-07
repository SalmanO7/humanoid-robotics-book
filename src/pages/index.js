import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Link from '@docusaurus/Link';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`${siteConfig.title}`} description="Physical AI & Humanoid Robotics Book">
      <main>
        <div className="container margin-vert--xl">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <h1 className="hero__title">{siteConfig.title}</h1>
              <p className="hero__subtitle">{siteConfig.tagline}</p>
              <div className="margin-vert--lg">
                <Link className="button button--primary button--lg" to="/docs/intro">
                  Read the Book
                </Link>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}