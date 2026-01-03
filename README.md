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

Set these in your Hugging Face Space settings (already configured):

```
GEMINI_API_KEY=your-google-gemini-api-key
QDRANT_URL=https://your-cluster.qdrant.cloud:6333
QDRANT_API_KEY=your-qdrant-api-key
NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/dbname
CORS_ORIGINS=https://your-frontend-url.com
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
