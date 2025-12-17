"""
Ingestion script for the RAG chatbot.
This script processes book content and stores it in the vector database.
"""
import os
import sys
from typing import List, Dict, Any
import asyncio

# Add the src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from services.rag_service import RAGService
from config import settings
from utils import logger


def read_book_content() -> List[Dict[str, Any]]:
    """
    Read book content from the docs directory.
    This is a simplified implementation that reads all MD/MDX files.
    In a real implementation, you would parse the book structure appropriately.
    """
    content_items = []
    docs_path = os.path.join(os.path.dirname(__file__), '../../../docs')
    
    if not os.path.exists(docs_path):
        logger.error(f"Docs directory does not exist: {docs_path}")
        return content_items
    
    for root, dirs, files in os.walk(docs_path):
        for file in files:
            if file.endswith(('.md', '.mdx')):
                file_path = os.path.join(root, file)
                relative_path = os.path.relpath(file_path, os.path.dirname(__file__))
                
                # Create a source page identifier (e.g., "/docs/intro" from "docs/intro.md")
                source_page = relative_path.replace('\\', '/').replace('.md', '').replace('.mdx', '')
                if source_page.startswith('../../../'):
                    source_page = '/' + source_page[7:]  # Remove the relative path prefix
                
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                    # Extract the first line as a section title if it's a heading
                    lines = content.split('\n')
                    section_title = "Untitled"
                    for line in lines:
                        if line.startswith('# '):
                            section_title = line[2:]  # Remove '# ' prefix
                            break
                        elif line.strip() != '':
                            # Use the first non-empty line as title if no heading found
                            section_title = line.strip()[:50]  # Limit length
                            break
                    
                    content_items.append({
                        'source_page': source_page,
                        'content': content,
                        'section_title': section_title
                    })
                    
                    logger.info(f"Read content from {file_path}: {len(content)} characters")
    
    return content_items


def main():
    """
    Main entry point for the ingestion script.
    """
    logger.info("Starting content ingestion process...")
    
    # Read book content
    content_items = read_book_content()
    
    if not content_items:
        logger.warning("No content found to ingest")
        return
    
    logger.info(f"Found {len(content_items)} content items to process")
    
    # Initialize the RAG service
    rag_service = RAGService()
    
    # Process and ingest content
    try:
        processed_count = rag_service.ingest_content(content_items)
        logger.info(f"Successfully processed and ingested {processed_count} content chunks")
    except Exception as e:
        logger.error(f"Error during ingestion: {e}")
        raise
    
    logger.info("Content ingestion process completed successfully")


if __name__ == "__main__":
    main()