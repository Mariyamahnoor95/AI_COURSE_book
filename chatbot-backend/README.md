---
title: Physical AI Robotics Chatbot Backend
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: docker
pinned: false
license: mit
---

# Physical AI & Humanoid Robotics Textbook Chatbot Backend

RAG-based chatbot backend using Google Gemini API for an interactive robotics textbook.

## Features

- **Gemini-powered RAG**: Uses Google Gemini 1.5 Flash for chat and text-embedding-004 for embeddings
- **Vector Search**: Qdrant Cloud for semantic search across textbook content
- **Chat History**: Neon Postgres for conversation persistence
- **Context Filtering**: Search across full textbook or specific modules/chapters

## API Endpoints

- `GET /health` - Health check
- `POST /api/chat` - Chat with the textbook assistant
- `GET /api/context/modules` - List available modules
- `GET /api/context/chapters` - List available chapters
- `GET /docs` - Interactive API documentation

## Environment Variables Required

Set these in your Hugging Face Space settings:

```
GEMINI_API_KEY=your-google-gemini-api-key
QDRANT_URL=https://your-cluster.qdrant.cloud:6333
QDRANT_API_KEY=your-qdrant-api-key
NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/dbname
CORS_ORIGINS=https://your-frontend-url.com,https://another-domain.com
```

## Setup Instructions

1. **Get API Keys**:
   - Gemini: https://aistudio.google.com/app/apikey
   - Qdrant Cloud: https://cloud.qdrant.io/
   - Neon: https://neon.tech/

2. **Run Setup Scripts** (one-time):
   ```bash
   python scripts/setup_db.py      # Create database tables
   python scripts/setup_qdrant.py  # Create vector collection
   python scripts/ingest_content.py # Embed textbook content
   ```

3. **Deploy to Hugging Face Spaces**

## Local Development

```bash
# Install dependencies
pip install -r requirements.txt

# Set up .env file
cp .env.example .env
# Edit .env with your API keys

# Run setup scripts
python scripts/setup_db.py
python scripts/setup_qdrant.py

# Start server
uvicorn src.main:app --reload --port 8000
```

## Tech Stack

- **Framework**: FastAPI 0.115+
- **LLM**: Google Gemini 1.5 Flash
- **Embeddings**: Gemini text-embedding-004 (768-dim)
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Postgres
- **Deployment**: Hugging Face Spaces (Docker SDK)

## License

MIT
