# ADR-001: Better-Auth for Authentication

**Date:** 2025-12-21
**Status:** Accepted

## Context

The Physical AI & Humanoid Robotics book requires user authentication with custom profile fields:
- Hardware background (GPU Workstation, Jetson Orin Nano, Cloud/Mac)
- Software experience level (beginner, intermediate, advanced)
- Personalization preferences for content display

Traditional authentication solutions (Auth0, Firebase Auth) don't easily support custom signup fields without complex workarounds. We needed a flexible, type-safe authentication system that integrates natively with our Next.js/React stack.

## Decision

We chose **Better-Auth** as our authentication framework.

Better-Auth is a modern authentication library built specifically for Next.js and React applications. It provides:
- TypeScript-first API with full type safety
- Built-in support for custom user fields via database schema
- Session management with cookie-based auth
- Social OAuth providers (Google, GitHub)
- Integration with PostgreSQL (our database choice)

## Consequences

### Positive

- **Custom Fields Native**: Hardware/software profiles stored directly in user table without joins
- **Type Safety**: Full TypeScript support prevents runtime auth errors
- **Next.js Integration**: Works seamlessly with App Router and Server Components
- **Flexible Schema**: Easy to extend user model as requirements evolve
- **Session Security**: HTTPOnly cookies prevent XSS attacks

### Negative

- **Newer Library**: Smaller community compared to Auth.js or Clerk
- **Documentation**: Less comprehensive than established solutions
- **Migration Risk**: Harder to migrate away if needed later
- **Learning Curve**: Team needs to learn Better-Auth patterns

### Neutral

- Requires PostgreSQL (Neon) - already part of our stack
- Self-hosted auth logic - more control but more responsibility

## Alternatives Considered

- **Supabase Auth**: Excellent solution but requires buying into entire Supabase ecosystem. We wanted flexibility to use Neon PostgreSQL and Vercel separately.

- **Clerk**: Feature-rich with beautiful UI components, but expensive for hobby/educational projects ($25/month). Better-Auth is free and open-source.

- **Auth.js (NextAuth.js)**: Industry standard with great community. However, custom fields require adapter customization and database joins, making personalization queries more complex.

- **Firebase Auth**: Google's solution, but poor TypeScript support and requires separate Firestore for user profiles, adding complexity.

## References

- Related specs: `specs/001-better-auth-signup/spec.md`, `specs/005-personalization-system/spec.md`
- Better-Auth docs: https://better-auth.com
- Implementation: `humanoid_robot_book/src/contexts/AuthContext.tsx`
