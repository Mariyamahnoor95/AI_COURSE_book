# Quickstart Guide

**Feature**: Physical AI & Humanoid Robotics Interactive Textbook
**Date**: 2025-12-17
**Phase**: Phase 1 - Design & Contracts
**Audience**: Developers, content authors, instructors

---

## Overview

This guide provides step-by-step instructions for:
1. **Local Development Setup** - Run textbook and chatbot locally
2. **Content Authoring Workflow** - Write and test new chapters
3. **Deployment Process** - Deploy to GitHub Pages and Render

---

## 1. Local Development Setup

### Prerequisites

Install the following before starting:

| Tool | Version | Purpose | Installation |
|------|---------|---------|--------------|
| **Node.js** | 18+ | Docusaurus runtime | [nodejs.org](https://nodejs.org/) |
| **Python** | 3.10+ | FastAPI backend | [python.org](https://www.python.org/) |
| **Git** | 2.30+ | Version control | [git-scm.com](https://git-scm.com/) |
| **Docker** | Optional | Local Qdrant instance | [docker.com](https://www.docker.com/) |

### Step 1: Clone Repository

```bash
git clone <repository-url>
cd hackathon1_book
```

### Step 2: Install Frontend Dependencies

```bash
# Install Docusaurus and React dependencies
npm install

# Verify installation
npm run docusaurus -- --version  # Should show Docusaurus 3.x
```

### Step 3: Install Backend Dependencies

```bash
# Navigate to backend directory
cd chatbot-backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Verify installation
python -c "import fastapi; print(fastapi.__version__)"  # Should show 0.104+
```

### Step 4: Configure Environment Variables

Create `.env` file in `chatbot-backend/`:

```bash
# chatbot-backend/.env
OPENAI_API_KEY=sk-proj-...  # Your OpenAI API key
QDRANT_URL=https://your-cluster.qdrant.cloud:6333  # Qdrant Cloud URL
QDRANT_API_KEY=your-qdrant-api-key
NEON_DATABASE_URL=postgresql://user:pass@ep-...neon.tech/dbname  # Neon Postgres URL
ENVIRONMENT=development
CORS_ORIGINS=http://localhost:3000
```

**Get API keys:**
- OpenAI: [platform.openai.com/api-keys](https://platform.openai.com/api-keys)
- Qdrant Cloud: [cloud.qdrant.io](https://cloud.qdrant.io/) (Free Tier)
- Neon Postgres: [neon.tech](https://neon.tech/) (Free Tier)

### Step 5: Run Content Ingestion

Populate vector database with textbook content:

```bash
# From chatbot-backend/ directory
cd scripts
python ingest_content.py --docs-dir ../../docs

# Expected output:
# ✓ Parsed 23 chapters
# ✓ Generated 230 chunks
# ✓ Embedded chunks (took ~45 seconds)
# ✓ Indexed to Qdrant
# Done! Vector DB ready.
```

### Step 6: Start Backend Server

```bash
# From chatbot-backend/ directory
uvicorn src.main:app --reload --port 8000

# Expected output:
# INFO:     Uvicorn running on http://127.0.0.1:8000
# INFO:     Application startup complete.
```

**Verify backend:**
- API docs: [http://localhost:8000/docs](http://localhost:8000/docs) (Swagger UI)
- Health check: `curl http://localhost:8000/health`

### Step 7: Start Frontend Server

In a new terminal:

```bash
# From repository root
npm start

# Expected output:
# [SUCCESS] Docusaurus website is running at:
# http://localhost:3000/
```

**Verify frontend:**
- Textbook: [http://localhost:3000](http://localhost:3000)
- Chatbot widget: Should appear in bottom-right corner (click to expand)

### Troubleshooting

**Port conflicts:**
```bash
# Backend using port 8000
lsof -ti:8000 | xargs kill -9  # Kill process on port 8000

# Frontend using port 3000
lsof -ti:3000 | xargs kill -9  # Kill process on port 3000
```

**CORS errors:**
- Verify `CORS_ORIGINS=http://localhost:3000` in `chatbot-backend/.env`
- Restart backend after `.env` changes

**Qdrant connection errors:**
- Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Verify Qdrant Cloud cluster is active (not hibernated)

---

## 2. Content Authoring Workflow

### Creating a New Chapter

**Step 1: Create Markdown file**

Follow naming convention: `chXX-topic-slug.md`

```bash
# Example: Create chapter in Module 1, Week 3
touch docs/module-01-ros2/week-03/ch03-new-topic.md
```

**Step 2: Add frontmatter**

```markdown
---
id: module-01-ros2/week-03/ch03-new-topic
title: New Topic Title
sidebar_label: New Topic
sidebar_position: 3
---

# New Topic Title

Your content here...
```

**Step 3: Write content**

Follow these conventions:

- **Headings**: Use H2 (`##`) for main sections, H3 (`###`) for subsections
- **Code blocks**: Use triple backticks with language identifier
  ```python
  # Python example
  import rclpy
  ```
- **Images**: Place in `static/img/` directory
  ```markdown
  ![Alt text](/img/module-01/diagram.png)
  ```
- **Internal links**: Use relative paths
  ```markdown
  See [ROS 2 Basics](./ch01-ros2-basics.md) for more details.
  ```

**Best practices:**
- Keep chapters 1,500-2,500 words (8-12 chunks)
- Use code examples with inline comments
- Add diagrams for complex architectures
- Link to official documentation for tools/libraries

**Step 4: Update sidebar**

Edit `sidebars.js` to add chapter to navigation:

```javascript
{
  type: 'category',
  label: 'Week 3',
  items: [
    'module-01-ros2/week-03/ch01-nodes-topics',
    'module-01-ros2/week-03/ch02-services-actions',
    'module-01-ros2/week-03/ch03-new-topic'  // Add new chapter
  ]
}
```

**Step 5: Test locally**

```bash
# Frontend auto-reloads, navigate to new chapter
open http://localhost:3000/docs/module-01-ros2/week-03/ch03-new-topic
```

**Step 6: Validate Markdown**

```bash
# Run Markdown linter
npm run lint:md

# Check for broken links
npm run check-links
```

**Step 7: Re-ingest content**

Update vector database with new chapter:

```bash
cd chatbot-backend/scripts
python ingest_content.py --docs-dir ../../docs --incremental

# Expected output:
# ✓ Detected 1 new/modified chapter
# ✓ Generated 10 new chunks
# ✓ Embedded and indexed
# Done!
```

**Step 8: Test chatbot**

1. Open chatbot widget in browser
2. Select module context ("Module 1: ROS 2")
3. Ask question related to new chapter
4. Verify response includes citation to new chapter

---

## 3. Deployment Process

### Deploying Textbook (GitHub Pages)

**Prerequisites:**
- GitHub repository with `main` branch
- GitHub Pages enabled (Settings → Pages → Source: `gh-pages` branch)

**Step 1: Configure Docusaurus**

Edit `docusaurus.config.js`:

```javascript
const config = {
  url: 'https://your-username.github.io',
  baseUrl: '/repository-name/',
  organizationName: 'your-username',
  projectName: 'repository-name',
  deploymentBranch: 'gh-pages',
  // ...
};
```

**Step 2: Build and deploy**

```bash
# Build static site
npm run build

# Deploy to GitHub Pages
GIT_USER=your-username npm run deploy

# Expected output:
# ✓ Built docs to build/
# ✓ Pushed to gh-pages branch
# ✓ Deployment complete
```

**Step 3: Verify deployment**

Visit: `https://your-username.github.io/repository-name/`

**Automated deployment (recommended):**

Create `.github/workflows/deploy-docs.yml`:

```yaml
name: Deploy Docs

on:
  push:
    branches: [main]
    paths:
      - 'docs/**'
      - 'src/**'
      - 'static/**'
      - 'docusaurus.config.js'
      - 'sidebars.js'

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

---

### Deploying Chatbot Backend (Render)

**Prerequisites:**
- Render account ([render.com](https://render.com/))
- GitHub repository connected to Render

**Step 1: Create Render Web Service**

1. Go to [Render Dashboard](https://dashboard.render.com/)
2. Click "New" → "Web Service"
3. Connect GitHub repository
4. Configure:
   - **Name**: `textbook-chatbot`
   - **Root Directory**: `chatbot-backend`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - **Plan**: Free (with 15-minute sleep)

**Step 2: Add environment variables**

In Render dashboard, add:
- `OPENAI_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `NEON_DATABASE_URL`
- `ENVIRONMENT=production`
- `CORS_ORIGINS=https://your-username.github.io`

**Step 3: Deploy**

Click "Create Web Service" → Render builds and deploys automatically.

**Step 4: Verify deployment**

- API docs: `https://textbook-chatbot.onrender.com/docs`
- Health check: `curl https://textbook-chatbot.onrender.com/health`

**Step 5: Update frontend**

Edit `src/components/ChatbotWidget/config.ts`:

```typescript
export const CHATBOT_API_URL =
  process.env.NODE_ENV === 'production'
    ? 'https://textbook-chatbot.onrender.com/v1'
    : 'http://localhost:8000/v1';
```

**Step 6: Redeploy frontend**

```bash
npm run deploy  # Rebuild and push to GitHub Pages
```

**Automated deployment (recommended):**

Render auto-deploys on push to `main` branch (if "Auto-Deploy" enabled).

---

## 4. Common Tasks

### Adding a New Module

1. **Create directory structure:**
   ```bash
   mkdir -p docs/module-05-new-topic/week-14
   ```

2. **Add chapters:**
   ```bash
   touch docs/module-05-new-topic/week-14/ch01-intro.md
   touch docs/module-05-new-topic/assessment-new-topic.md
   ```

3. **Update `sidebars.js`:**
   ```javascript
   {
     type: 'category',
     label: 'Module 5: New Topic (Week 14)',
     items: [/* ... */]
   }
   ```

4. **Re-ingest content** (see Content Authoring Step 7)

### Updating Chunking Parameters

Edit `chatbot-backend/src/utils/chunking.py`:

```python
# Adjust chunk size
CHUNK_SIZE = 500  # tokens (default: 400-600)
OVERLAP = 75      # tokens (default: 60)
```

Re-run ingestion after changes.

### Monitoring API Usage

**OpenAI costs:**
```bash
# Check token usage (last 30 days)
curl https://api.openai.com/v1/usage?date=2025-12 \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

**Qdrant storage:**
- Dashboard: [cloud.qdrant.io](https://cloud.qdrant.io/)
- Check "Collections" → "textbook_chunks" → Storage used

**Neon database size:**
```sql
-- Connect to Neon Postgres
psql $NEON_DATABASE_URL

-- Check database size
SELECT pg_size_pretty(pg_database_size(current_database()));
```

### Backup and Recovery

**Backup vector database:**
```bash
# Export Qdrant snapshot
curl -X POST "https://your-cluster.qdrant.cloud:6333/collections/textbook_chunks/snapshots" \
  -H "api-key: $QDRANT_API_KEY"

# Download snapshot
curl "https://your-cluster.qdrant.cloud:6333/collections/textbook_chunks/snapshots/{snapshot_name}" \
  -H "api-key: $QDRANT_API_KEY" > backup.snapshot
```

**Backup chat history:**
```bash
# Export Neon database
pg_dump $NEON_DATABASE_URL > chathistory_backup.sql

# Restore
psql $NEON_DATABASE_URL < chathistory_backup.sql
```

---

## 5. Performance Optimization

### Frontend (Docusaurus)

**Image optimization:**
```bash
# Compress images in static/img/
npm install -g imagemin-cli
imagemin static/img/*.png --out-dir=static/img/optimized
```

**Bundle analysis:**
```bash
npm run build -- --bundle-analyzer
```

### Backend (FastAPI)

**Enable caching:**

Edit `chatbot-backend/src/services/rag.py`:

```python
from functools import lru_cache

@lru_cache(maxsize=100)
def embed_query(query: str) -> List[float]:
    # Cache frequent queries
    return openai.embeddings.create(...)
```

**Database connection pooling:**

Edit `chatbot-backend/src/services/chat_history.py`:

```python
from psycopg_pool import ConnectionPool

pool = ConnectionPool(
    conninfo=os.getenv("NEON_DATABASE_URL"),
    min_size=2,
    max_size=10
)
```

---

## 6. Troubleshooting

### Chatbot not responding

1. **Check backend logs** (Render dashboard or local terminal)
2. **Verify API keys** in `.env`
3. **Test RAG retrieval:**
   ```bash
   curl -X POST http://localhost:8000/v1/chat \
     -H "Content-Type: application/json" \
     -d '{"message": "test query", "context": {"mode": "full_textbook", "selected_module_ids": [], "selected_chapter_ids": []}}'
   ```

### Slow response times (>3 seconds)

1. **Check OpenAI API status:** [status.openai.com](https://status.openai.com/)
2. **Reduce chunk retrieval:**
   - Edit `chatbot-backend/src/services/rag.py`
   - Lower `top_k` parameter (default: 5-10)
3. **Enable query caching** (see Performance Optimization above)

### Content not appearing in chatbot

1. **Verify ingestion completed:** Check `ingest_content.py` output
2. **Check Qdrant collection:**
   ```bash
   curl "https://your-cluster.qdrant.cloud:6333/collections/textbook_chunks" \
     -H "api-key: $QDRANT_API_KEY"
   ```
3. **Re-run ingestion:** `python ingest_content.py --docs-dir ../../docs --force`

---

## 7. Development Best Practices

### Git Workflow

```bash
# Create feature branch
git checkout -b feature/add-module-5

# Make changes, commit frequently
git add docs/module-05-new-topic/
git commit -m "Add Module 5: New Topic chapters 1-3"

# Push and create PR
git push origin feature/add-module-5
```

### Code Review Checklist

- [ ] Markdown linting passes (`npm run lint:md`)
- [ ] No broken links (`npm run check-links`)
- [ ] Chapter added to `sidebars.js`
- [ ] Images optimized (<500KB per image)
- [ ] Code examples tested and functional
- [ ] Chatbot responds accurately to chapter content

### Testing Strategy

**Unit tests (backend):**
```bash
cd chatbot-backend
pytest tests/test_rag.py  # RAG accuracy tests
pytest tests/test_api.py  # API endpoint tests
```

**End-to-end tests (frontend):**
```bash
npm run test:e2e  # Playwright tests (navigation + chatbot)
```

---

## References

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [OpenAI Agents SDK](https://openai.github.io/openai-agents-python/)
- [ChatKit SDK](https://platform.openai.com/docs/guides/chatkit)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Neon Postgres Guides](https://neon.tech/docs/introduction)
- [data-model.md](./data-model.md) - Entity definitions
- [contracts/chatbot-api.yaml](./contracts/chatbot-api.yaml) - API specification
- [research.md](./research.md) - Phase 0 technical decisions
