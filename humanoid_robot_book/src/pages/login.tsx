import React from 'react';
import Layout from '@theme/Layout';
import LoginForm from '../components/Auth/LoginForm';

export default function LoginPage(): JSX.Element {
  return (
    <Layout title="Login" description="Login to your account">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '80vh',
        padding: '20px'
      }}>
        <LoginForm />
      </main>
    </Layout>
  );
}