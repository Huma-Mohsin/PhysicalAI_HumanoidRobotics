# Deploy Book Skill

**Description**: Automated deployment workflow for the Physical AI & Humanoid Robotics book

**Version**: 1.0.0

**Author**: Physical AI Project Team

---

## What This Skill Does

Automates the complete deployment process for the humanoid robotics book:
1. Build frontend (Docusaurus)
2. Update embeddings (Qdrant vector store)
3. Deploy to Vercel
4. Verify deployment

---

## Usage

```bash
# From project root
claude skill deploy-book
```

Or with Claude Code:
```
Use the deploy-book skill to deploy the latest changes
```

---

## Steps Executed

### 1. Build Frontend
```bash
cd humanoid_robot_book
npm install
npm run build
```

### 2. Update Embeddings
```bash
cd backend
python scripts/embed_book_content.py
python scripts/validate_embeddings.py
```

### 3. Git Commit & Push
```bash
git add .
git commit -m "deploy: Update book content and embeddings"
git push origin main
```

### 4. Verify Deployment
- Check Vercel dashboard
- Test live URL
- Verify RAG chatbot responses

---

## Prerequisites

- Node.js 18+ installed
- Python 3.10+ with backend dependencies
- Vercel CLI configured
- Environment variables set:
  - `COHERE_API_KEY` or `OPENAI_API_KEY`
  - `QDRANT_URL`
  - `QDRANT_API_KEY`

---

## Success Criteria

- ✅ Frontend builds without errors
- ✅ Embeddings updated in Qdrant
- ✅ Git push successful
- ✅ Vercel deployment live
- ✅ RAG queries return updated content

---

## Example Output

```
[INFO] Building frontend...
[SUCCESS] Build complete: humanoid_robot_book/build/

[INFO] Updating embeddings...
[INFO] Found 7 MDX files
[SUCCESS] Embedded 226 chunks to Qdrant

[INFO] Deploying to Vercel...
[SUCCESS] Deployed to: https://your-book.vercel.app

✅ Deployment complete!
```

---

## Troubleshooting

**Build fails**: Check Node.js version (`node --version`)
**Embeddings fail**: Verify API keys in `.env`
**Vercel timeout**: Increase function timeout in `vercel.json`

---

## Tags
`deployment`, `automation`, `vercel`, `rag`, `embeddings`
