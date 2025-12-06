# Deployment Guide: Physical AI RAG Platform

Production deployment checklist and instructions.

## Pre-Deployment Checklist

### Security

- [ ] All `.env` files contain production credentials (not example values)
- [ ] `SECRET_KEY` is cryptographically secure (generated with `openssl rand -hex 32`)
- [ ] `DEBUG=false` in production environment
- [ ] CORS_ORIGINS configured for production domains only
- [ ] Rate limiting enabled (`RATE_LIMIT_ENABLED=true`)
- [ ] HTTPS enabled for all endpoints
- [ ] API keys rotated from development values
- [ ] Database uses SSL connections (`sslmode=require`)

### Configuration

- [ ] Environment variables configured:
  - [ ] `DATABASE_URL` (Neon Postgres production)
  - [ ] `QDRANT_URL` and `QDRANT_API_KEY` (Qdrant Cloud)
  - [ ] `OPENAI_API_KEY` (OpenAI production key)
  - [ ] `SECRET_KEY` (unique for production)
- [ ] Log level set to `INFO` or `WARNING`
- [ ] Log format set to `json` for structured logging
- [ ] API host and port configured (`0.0.0.0:8000`)

### Database

- [ ] Neon Postgres database created
- [ ] Alembic migrations applied: `alembic upgrade head`
- [ ] Database connection pool sized appropriately
- [ ] Database backups configured (Neon automatic backups)
- [ ] Read replicas configured (if needed)

### Vector Database

- [ ] Qdrant Cloud cluster created
- [ ] Collections initialized:
  - [ ] `content_embeddings_en`
  - [ ] `content_embeddings_ur`
- [ ] Content embedded: `python scripts/embed_content.py`
- [ ] Vector count verified (47+ chunks for English)

### Testing

- [ ] All tests passing: `pytest --cov=src`
- [ ] Coverage >= 70%
- [ ] Integration tests verified with production-like environment
- [ ] Load testing completed (100+ concurrent users)
- [ ] Security scan completed (no critical vulnerabilities)

### Documentation

- [ ] API documentation generated: `/docs` endpoint
- [ ] README.md updated with production URLs
- [ ] QUICKSTART.md reflects production setup
- [ ] Environment variable template (`.env.example`) current

---

## Deployment Options

### Option 1: Docker Compose (Recommended for Quick Deploy)

```bash
# Build and run services
docker-compose up --build -d

# Verify services are running
docker-compose ps

# View logs
docker-compose logs -f

# Services available at:
# - Backend: http://localhost:8000
# - Frontend: http://localhost:3000
```

### Option 2: Render.com (Backend) + Vercel (Frontend)

#### Backend on Render

1. **Create Web Service**:
   - Connect GitHub repository
   - Build command: `pip install -r backend/requirements.txt`
   - Start command: `cd backend && python -m src.api.main`
   - Environment: Python 3.11

2. **Configure Environment Variables**:
   ```
   DATABASE_URL=postgresql://...
   QDRANT_URL=https://...
   QDRANT_API_KEY=...
   OPENAI_API_KEY=sk-...
   SECRET_KEY=...
   ENVIRONMENT=production
   LOG_LEVEL=INFO
   LOG_FORMAT=json
   ```

3. **Custom Domain** (optional):
   - Add custom domain in Render dashboard
   - Update DNS CNAME record

#### Frontend on Vercel

1. **Import Project**:
   - Connect GitHub repository
   - Root directory: `frontend`
   - Framework: Docusaurus

2. **Build Settings**:
   - Build command: `npm run build`
   - Output directory: `build`
   - Install command: `npm install`

3. **Environment Variables**:
   ```
   REACT_APP_API_URL=https://your-backend.onrender.com
   ```

4. **Custom Domain** (optional):
   - Add domain in Vercel dashboard
   - Configure DNS

### Option 3: AWS (Production-Grade)

#### Backend on EC2/ECS

1. **EC2 Instance**:
   - Ubuntu 22.04 LTS
   - t3.medium or larger
   - Security group: Allow ports 80, 443, 8000

2. **Setup**:
   ```bash
   # SSH into instance
   ssh ubuntu@your-instance-ip

   # Install dependencies
   sudo apt update
   sudo apt install python3.11 python3-pip nginx

   # Clone repository
   git clone https://github.com/your-repo/PhysicalAI_HumanoidRobotics.git
   cd PhysicalAI_HumanoidRobotics/backend

   # Install requirements
   python3.11 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt

   # Configure environment
   cp .env.example .env
   nano .env  # Add production credentials

   # Run migrations
   alembic upgrade head

   # Test backend
   python -m src.api.main
   ```

3. **Process Manager (Supervisor)**:
   ```ini
   # /etc/supervisor/conf.d/physicalai-api.conf
   [program:physicalai-api]
   command=/home/ubuntu/PhysicalAI_HumanoidRobotics/backend/venv/bin/python -m src.api.main
   directory=/home/ubuntu/PhysicalAI_HumanoidRobotics/backend
   user=ubuntu
   autostart=true
   autorestart=true
   stderr_logfile=/var/log/physicalai-api.err.log
   stdout_logfile=/var/log/physicalai-api.out.log
   environment=PATH="/home/ubuntu/PhysicalAI_HumanoidRobotics/backend/venv/bin"
   ```

4. **Nginx Reverse Proxy**:
   ```nginx
   # /etc/nginx/sites-available/physicalai-api
   server {
       listen 80;
       server_name api.yoursite.com;

       location / {
           proxy_pass http://127.0.0.1:8000;
           proxy_set_header Host $host;
           proxy_set_header X-Real-IP $remote_addr;
           proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
           proxy_set_header X-Forwarded-Proto $scheme;
       }
   }
   ```

5. **SSL with Let's Encrypt**:
   ```bash
   sudo apt install certbot python3-certbot-nginx
   sudo certbot --nginx -d api.yoursite.com
   ```

#### Frontend on S3 + CloudFront

1. **Build Frontend**:
   ```bash
   cd frontend
   npm install
   npm run build
   ```

2. **Upload to S3**:
   ```bash
   aws s3 sync build/ s3://your-bucket-name --delete
   ```

3. **CloudFront Configuration**:
   - Origin: S3 bucket
   - Default root object: `index.html`
   - Error pages: 404 → `/index.html` (for SPA routing)
   - SSL: Use AWS Certificate Manager

---

## Post-Deployment Verification

### Health Checks

```bash
# Backend health
curl https://api.yoursite.com/health

# Expected response:
{
  "status": "healthy",
  "database": "connected",
  "qdrant": "connected",
  "timestamp": "2025-12-07T..."
}

# Chat health
curl https://api.yoursite.com/chat/health
```

### Smoke Tests

1. **Frontend loads**: Visit https://yoursite.com
2. **Authentication works**:
   - Sign up new user
   - Sign in with credentials
   - Access protected routes

3. **RAG chatbot works**:
   - Open chat interface
   - Ask: "How do I create a ROS 2 node?"
   - Verify response is contextual

4. **Feedback system**:
   - Submit feedback (thumbs up/down)
   - View analytics page
   - Verify stats update

### Performance Verification

```bash
# Load test
ab -n 1000 -c 10 https://api.yoursite.com/health

# Expected:
# - Requests per second: >100
# - 95th percentile: <500ms
# - Failed requests: 0
```

---

## Monitoring and Logging

### Structured Logging

Logs are output in JSON format for easy parsing:

```json
{
  "timestamp": "2025-12-07T12:00:00.000Z",
  "level": "INFO",
  "app": "Physical AI & Humanoid Robotics RAG Platform API",
  "environment": "production",
  "module": "main",
  "function": "lifespan",
  "line": 45,
  "message": "API server ready"
}
```

### Log Aggregation

**Option 1: CloudWatch (AWS)**
```bash
# Install CloudWatch agent
sudo apt install amazon-cloudwatch-agent

# Configure log streams
# API logs → /aws/ec2/physicalai-api
# Error logs → /aws/ec2/physicalai-api-errors
```

**Option 2: Datadog**
```bash
# Install Datadog agent
DD_API_KEY=your-key bash -c "$(curl -L https://s3.amazonaws.com/dd-agent/scripts/install_script.sh)"

# Configure log collection
# Edit /etc/datadog-agent/conf.d/python.d/conf.yaml
```

**Option 3: ELK Stack** (self-hosted)
- Elasticsearch: Store logs
- Logstash: Parse and transform
- Kibana: Visualize and search

### Metrics and Alerts

**Key Metrics to Monitor**:
- Request rate (req/s)
- Error rate (%)
- Response time (p50, p95, p99)
- Database connection pool usage
- Qdrant query latency
- OpenAI API latency
- Rate limit hits

**Alert Thresholds**:
- Error rate > 5%: Warning
- Error rate > 10%: Critical
- p95 response time > 2s: Warning
- p95 response time > 5s: Critical
- Rate limit hits > 100/min: Investigate

---

## Rollback Procedure

### Database Rollback

```bash
cd backend

# Rollback one migration
alembic downgrade -1

# Rollback to specific version
alembic downgrade <revision_id>
```

### Application Rollback

**Docker Compose**:
```bash
# Use previous image tag
docker-compose down
docker-compose -f docker-compose.yml -f docker-compose.prod.yml up -d --build
```

**Render**:
- Go to Render dashboard
- Select service
- Navigate to "Manual Deploy" tab
- Deploy previous commit

**Vercel**:
- Go to Vercel dashboard
- Select project
- Go to "Deployments"
- Promote previous deployment

---

## Scaling Considerations

### Horizontal Scaling (Backend)

```yaml
# docker-compose.scale.yml
services:
  backend:
    deploy:
      replicas: 3
    environment:
      - WORKERS=4  # Uvicorn workers per container

  nginx:
    image: nginx:alpine
    volumes:
      - ./nginx-lb.conf:/etc/nginx/nginx.conf
    depends_on:
      - backend
```

### Database Scaling

- **Read Replicas**: Configure Neon Postgres read replicas
- **Connection Pooling**: Use PgBouncer for connection management
- **Caching**: Add Redis for frequently accessed data

### Vector Database Scaling

- **Qdrant Clustering**: Use Qdrant Cloud cluster mode
- **Horizontal Pods**: Scale Qdrant horizontally
- **Sharding**: Partition embeddings by language/module

---

## Backup and Disaster Recovery

### Database Backups

**Neon Postgres** (automatic):
- Point-in-time recovery (7 days)
- Daily snapshots
- Restore via Neon dashboard

**Manual Backup**:
```bash
# Export database
pg_dump $DATABASE_URL > backup_$(date +%Y%m%d).sql

# Restore
psql $DATABASE_URL < backup_20251207.sql
```

### Vector Database Backups

```bash
# Export Qdrant collection
curl -X GET "https://your-cluster.qdrant.io/collections/content_embeddings_en/snapshots" \
  -H "api-key: $QDRANT_API_KEY"

# Create snapshot
curl -X POST "https://your-cluster.qdrant.io/collections/content_embeddings_en/snapshots" \
  -H "api-key: $QDRANT_API_KEY"
```

### Application Code

- **Git**: Version controlled on GitHub
- **Container Images**: Tagged and stored in registry
- **Configuration**: Environment variables documented in `.env.example`

---

## Security Hardening

### Production Security Checklist

- [ ] HTTPS enforced (HTTP redirects to HTTPS)
- [ ] Strong TLS configuration (TLS 1.2+)
- [ ] HTTP security headers configured:
  - [ ] `X-Content-Type-Options: nosniff`
  - [ ] `X-Frame-Options: DENY`
  - [ ] `X-XSS-Protection: 1; mode=block`
  - [ ] `Strict-Transport-Security: max-age=31536000`
- [ ] Rate limiting enabled (100 req/min per IP)
- [ ] CORS origins restricted to production domains
- [ ] SQL injection prevention (SQLAlchemy ORM)
- [ ] XSS prevention (request validation middleware)
- [ ] Secrets stored in environment variables (not code)
- [ ] Database credentials rotated regularly
- [ ] API keys have IP restrictions (where possible)

### Dependency Security

```bash
# Scan for vulnerabilities
pip install safety
safety check -r requirements.txt

# Update dependencies
pip install --upgrade -r requirements.txt
pip freeze > requirements.txt

# Audit npm packages (frontend)
cd frontend
npm audit
npm audit fix
```

---

## Cost Optimization

### Service Tiers

**Development** (~$25/month):
- Neon Postgres: Free tier (0.5 GB)
- Qdrant Cloud: Free tier (1 GB)
- OpenAI: $5-10/month (moderate usage)
- Render: Free tier (slow spin-up)
- Vercel: Free tier

**Production** (~$100-200/month):
- Neon Postgres: Pro ($19/month + usage)
- Qdrant Cloud: Standard ($40/month)
- OpenAI: $50-100/month (varies with usage)
- Render: Starter ($7/month per service)
- Vercel: Pro ($20/month)
- CloudFlare: Free (CDN and DDoS protection)

### Cost Reduction Tips

1. **Cache OpenAI responses** (Redis TTL: 1 hour)
2. **Use embedding cache** for repeat queries
3. **Optimize database queries** (indexes, EXPLAIN ANALYZE)
4. **Compress static assets** (gzip, Brotli)
5. **Use CDN** for frontend assets (CloudFlare, CloudFront)
6. **Monitor API usage** (set spend alerts)

---

## Troubleshooting

### "Service Unavailable" errors

1. Check service health: `curl https://api.yoursite.com/health`
2. View logs: `docker-compose logs -f backend`
3. Verify database connection: Check `DATABASE_URL`
4. Restart service: `docker-compose restart backend`

### "Rate limit exceeded"

1. Verify rate limit settings: `RATE_LIMIT_REQUESTS`, `RATE_LIMIT_WINDOW`
2. Check client IP (behind proxy): Ensure `X-Forwarded-For` header
3. Adjust limits for production load
4. Implement user-specific rate limits (authenticated users)

### "Slow response times"

1. Check database query performance: Enable query logging
2. Monitor OpenAI API latency: Add timing logs
3. Verify Qdrant connection: Check network latency
4. Add caching: Redis for frequently accessed data
5. Scale horizontally: Add more backend instances

---

**Last Updated**: 2025-12-07
**Deployment Version**: 1.0.0
**Platform**: Production-ready
