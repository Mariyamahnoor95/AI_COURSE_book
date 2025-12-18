# Render Deployment Instructions

This document explains how to deploy the FastAPI backend to Render Free Tier.

## Prerequisites

1. **Render Account**: Sign up at https://render.com/
2. **External Services**:
   - Qdrant Cloud collection created (run `setup_qdrant.py`)
   - Neon Postgres database created (run `setup_db.py`)
   - OpenAI API key obtained

## Deployment Methods

### Method 1: Render Blueprint (Recommended)

1. **Fork/Push Repository to GitHub**
   ```bash
   git add .
   git commit -m "Add Render configuration"
   git push origin main
   ```

2. **Create Render Web Service from Blueprint**
   - Go to https://dashboard.render.com/
   - Click "New" → "Blueprint"
   - Connect your GitHub repository
   - Select the repository
   - Render will detect `chatbot-backend/render.yaml`

3. **Set Environment Variables (Secrets)**
   - In Render dashboard, go to your service
   - Navigate to "Environment" tab
   - Add the following secrets:
     ```
     OPENAI_API_KEY=sk-proj-...
     QDRANT_URL=https://your-cluster.qdrant.cloud:6333
     QDRANT_API_KEY=your-qdrant-api-key
     NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/dbname
     ```

4. **Deploy**
   - Click "Create Web Service"
   - Render will build and deploy automatically
   - First deploy takes ~5-10 minutes

### Method 2: Manual Setup

1. **Create New Web Service**
   - Go to https://dashboard.render.com/
   - Click "New" → "Web Service"
   - Connect GitHub repository
   - Select repository and branch (main)

2. **Configure Service**
   - **Name**: `physical-ai-chatbot-api`
   - **Region**: Oregon (or closest to users)
   - **Branch**: `main`
   - **Root Directory**: `chatbot-backend`
   - **Environment**: Python 3
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - **Plan**: Free

3. **Add Environment Variables**
   - Same as Method 1, step 3

4. **Deploy**
   - Click "Create Web Service"

## Free Tier Limitations

- **Sleep Mode**: Service sleeps after 15 minutes of inactivity
- **Cold Start**: 30-60 seconds to wake up (handled by T014b)
- **Hours**: 750 hours/month (31.25 days × 24 hours = plenty for pilot)
- **Bandwidth**: 100GB/month outbound

## Auto-Deployment

Render automatically redeploys when you push to the `main` branch on GitHub.

To disable auto-deploy:
1. Go to service settings
2. Navigate to "Build & Deploy" tab
3. Toggle "Auto-Deploy" off

## Monitoring

**Service URL**: `https://physical-ai-chatbot-api.onrender.com`

**Health Check**: `GET https://physical-ai-chatbot-api.onrender.com/health`

**Logs**: View in Render dashboard → "Logs" tab

## Troubleshooting

### Service Won't Start

1. Check logs for errors
2. Verify all environment variables are set
3. Ensure `requirements.txt` includes all dependencies
4. Confirm Python version compatibility (3.10+)

### Cold Start Issues

- This is expected behavior on Free tier
- Frontend handles cold starts with loading state (T014b)
- Consider upgrading to Starter plan ($7/month) for always-on service

### Database Connection Errors

1. Verify `NEON_DATABASE_URL` is correct
2. Check Neon project is active (Free tier suspends after 7 days inactivity)
3. Run `setup_db.py` if tables don't exist

### CORS Errors

1. Verify `CORS_ORIGINS` includes your GitHub Pages URL
2. Check FastAPI CORS middleware configuration in `src/main.py`

## Updating the Service

To update configuration:

1. Edit `render.yaml`
2. Commit and push changes
3. Render will automatically redeploy

To update code:

1. Make changes to Python files
2. Commit and push to `main`
3. Render will automatically build and redeploy

## Cost Optimization

**Free Tier is sufficient for pilot deployment:**
- Expected API calls: ~1,000/month (students × queries)
- OpenAI costs: ~$20-50/month (text-embedding-3-small: $0.00002/1K tokens)
- Qdrant: Free 1GB (using ~1.5MB)
- Neon: Free 0.5GB (using ~10MB)
- Render: Free 750 hours/month

**Total estimated cost: $20-50/month** (OpenAI only)
