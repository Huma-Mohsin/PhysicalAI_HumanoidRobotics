# ADR-005: Vercel for Deployment

**Date:** 2025-12-21
**Status:** Accepted

## Context

The project requires hosting for:
- **Frontend**: Docusaurus static site (React SPA)
- **Backend**: FastAPI Python serverless functions
- **Database**: PostgreSQL (Neon) for user auth
- **Vector DB**: Qdrant Cloud for embeddings

Requirements:
- Zero-downtime deployments from GitHub commits
- Serverless/auto-scaling (traffic varies)
- HTTPS and custom domain support
- CDN for fast global access
- Free tier sufficient for educational project
- Support for both Node.js (frontend) and Python (backend)

## Decision

We chose **Vercel** as our deployment platform for both frontend and backend.

Deployment structure:
- Frontend: `humanoid_robot_book/` → https://humanoidrobotbook.vercel.app
- Backend: `backend/` → https://humanoid-robotics-backend.vercel.app/api/*
- Auto-deploy: GitHub main branch push triggers builds
- Serverless functions: FastAPI routes as Python serverless functions

## Consequences

### Positive

- **Zero Config**: Detected Docusaurus and Next.js automatically
- **Git Integration**: Auto-deploy on `git push` to main branch
- **Dual Runtime**: Supports both Node.js (frontend) and Python (backend) in same project
- **Free Tier**: Generous limits (100GB bandwidth, 100 serverless invocations/day)
- **Edge Network**: Global CDN for < 100ms page loads
- **Preview Deployments**: Each PR gets unique URL for testing
- **Environment Variables**: Secure secret management for API keys
- **Analytics**: Built-in Web Vitals and traffic analytics

### Negative

- **Vendor Lock-in**: Vercel-specific configurations (`vercel.json`) not portable
- **Cold Starts**: Python serverless functions have 300-500ms cold start
- **Timeout Limits**: 10-second max execution time (free tier) - acceptable for RAG queries
- **Build Time**: Docusaurus builds take 2-3 minutes
- **No Persistent Storage**: Serverless functions are stateless (requires external DB)

### Neutral

- Requires separate Neon PostgreSQL (not colocated with compute)
- No SSH access (serverless only, no VMs)

## Alternatives Considered

- **Netlify**: Similar to Vercel with great DX. Rejected because:
  - Weaker Python serverless function support (better for Node.js)
  - Slower build times for Docusaurus in our testing
  - Vercel has better Next.js/React ecosystem integration

- **AWS Lambda + S3 + CloudFront**: Most flexible cloud option. Rejected because:
  - Complex setup (3+ AWS services to configure)
  - No auto-deploy from GitHub without CodePipeline setup
  - Overkill for educational project
  - Higher cost for equivalent traffic

- **Railway**: Modern PaaS with great developer experience. Rejected because:
  - Free tier only provides $5/month credit (runs out quickly)
  - Focused on long-running services, not serverless
  - Better for traditional backends, not JAMstack

- **GitHub Pages**: Free static hosting. Rejected because:
  - Only supports static sites (no backend serverless functions)
  - Would need separate hosting for FastAPI backend
  - Slower global CDN than Vercel

- **Self-hosted (DigitalOcean/Linode)**: Full control with VPS. Rejected because:
  - Manual server maintenance (security patches, scaling)
  - No auto-deploy (would need CI/CD setup)
  - Fixed costs (~$5-10/month) even with zero traffic
  - Overkill for a book/chatbot project

## References

- Vercel docs: https://vercel.com/docs
- Frontend deployment: https://humanoidrobotbook.vercel.app
- Backend deployment: https://humanoid-robotics-backend.vercel.app
- Configuration: `vercel.json`, `backend/vercel.json`
- GitHub integration: Auto-deploy on push to `main` branch
