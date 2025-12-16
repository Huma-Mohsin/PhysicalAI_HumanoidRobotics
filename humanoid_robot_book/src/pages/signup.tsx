import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/Auth/SignupForm';

export default function SignupPage(): JSX.Element {
  return (
    <Layout title="Sign Up" description="Create a new account">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '80vh',
        padding: '20px'
      }}>
        <SignupForm />
      </main>
    </Layout>
  );
}