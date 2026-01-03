from contextlib import AsyncExitStack
from qdrant_client import QdrantClient
import cohere
import os


class ToolContext:
    def __init__(self):
        self.stack = AsyncExitStack()
        self.qdrant = None
        self.cohere = None

    async def __aenter__(self):
        print("ENTER TOOL CONTEXT")

        self.qdrant = QdrantClient(
            url=os.environ["QDRANT_URL"],
            api_key=os.environ["QDRANT_API_KEY"],
        )

        self.cohere = cohere.Client(
            api_key=os.environ["COHERE_API_KEY"]
        )

        return self

    async def __aexit__(self, *args):
        print("EXIT TOOL CONTEXT")
        await self.stack.aclose()


from agents import function_tool
import asyncio


@function_tool
def search_knowledge_base(query: str) -> str:
    async def _run():
        async with ToolContext() as ctx:
            print(f"🔎 Query: {query}")

            # Embed query
            embedding = ctx.cohere.embed(
                texts=[query],
                model="embed-v4.0",
                input_type="search_query"
            ).embeddings[0]

            # Vector search
            results = ctx.qdrant.query_points(
                collection_name="robotics_textbook_v1",
                query=embedding,
                limit=5,
                score_threshold=0.4
            ).points

            if not results:
                return "No relevant content found."

            return "\n".join(
                f"- {r.payload.get('text','')}"
                for r in results
            )

    return asyncio.run(_run())
