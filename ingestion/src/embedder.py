from typing import List, Dict, Any
from src.services.embedding_service import EmbeddingService
from src.services.vector_service import VectorService
from ingestion.src.parser import ContentParser
from ingestion.src.chunker import TextChunker
from src.utils import logger


class IngestionPipeline:
    def __init__(self):
        self.parser = ContentParser()
        self.chunker = TextChunker(chunk_size=1000, overlap_size=200)
        self.embedding_service = EmbeddingService()
        self.vector_service = VectorService()
    
    def process_book_content(self, source_directory: str) -> int:
        """
        Process all book content from a directory: parse, chunk, embed, and store.
        
        Args:
            source_directory: Path to the directory containing book content (MD/MDX files)
            
        Returns:
            Number of chunks processed and stored
        """
        try:
            # Parse all content in the directory
            logger.info(f"Starting to parse content from {source_directory}")
            content_items = self.parser.parse_directory(source_directory)
            logger.info(f"Parsed {len(content_items)} content items")
            
            if not content_items:
                logger.warning("No content found to process")
                return 0
            
            # Process each content item
            total_chunks = 0
            chunks_to_store = []
            
            for item in content_items:
                source_page = item['source_page']
                content = item['content']
                section_title = item['section_title']
                
                logger.info(f"Chunking content for {source_page}")
                
                # Chunk the content
                chunks = self.chunker.chunk_by_sentence(
                    text=content,
                    source_page=source_page,
                    section_title=section_title
                )
                
                # Generate embeddings for each chunk
                for chunk in chunks:
                    try:
                        # Generate embedding for the chunk content
                        embedding = self.embedding_service.generate_embedding(chunk['content'])
                        
                        # Add embedding to chunk
                        chunk['embedding'] = embedding
                        
                        # Add to storage list
                        chunks_to_store.append(chunk)
                        total_chunks += 1
                        
                    except Exception as e:
                        logger.error(f"Error generating embedding for chunk {chunk['chunk_id']}: {e}")
                        continue
            
            logger.info(f"Generated embeddings for {len(chunks_to_store)} chunks")
            
            # Store all chunks in vector database
            if chunks_to_store:
                self.vector_service.add_chunks(chunks_to_store)
            
            logger.info(f"Successfully stored {len(chunks_to_store)} chunks in vector database")
            return total_chunks
            
        except Exception as e:
            logger.error(f"Error in ingestion pipeline: {e}")
            raise
    
    def process_single_content(self, content: str, source_page: str, section_title: str) -> int:
        """
        Process a single piece of content: chunk, embed, and store.
        
        Args:
            content: The content to process
            source_page: The source page identifier
            section_title: The section title
            
        Returns:
            Number of chunks processed and stored
        """
        try:
            # Chunk the content
            chunks = self.chunker.chunk_by_sentence(
                text=content,
                source_page=source_page,
                section_title=section_title
            )
            
            # Generate embeddings for each chunk
            chunks_to_store = []
            for chunk in chunks:
                try:
                    # Generate embedding for the chunk content
                    embedding = self.embedding_service.generate_embedding(chunk['content'])
                    
                    # Add embedding to chunk
                    chunk['embedding'] = embedding
                    
                    # Add to storage list
                    chunks_to_store.append(chunk)
                    
                except Exception as e:
                    logger.error(f"Error generating embedding for chunk {chunk['chunk_id']}: {e}")
                    continue
            
            # Store all chunks in vector database
            if chunks_to_store:
                self.vector_service.add_chunks(chunks_to_store)
            
            logger.info(f"Successfully stored {len(chunks_to_store)} chunks in vector database from single content")
            return len(chunks_to_store)
            
        except Exception as e:
            logger.error(f"Error in single content ingestion: {e}")
            raise


# Example usage function
def run_ingestion_pipeline(source_directory: str):
    """
    Run the full ingestion pipeline on a directory of book content.
    
    Args:
        source_directory: Path to the directory containing book content (MD/MDX files)
    """
    pipeline = IngestionPipeline()
    total_chunks = pipeline.process_book_content(source_directory)
    print(f"Ingestion complete. Processed {total_chunks} chunks.")


# For direct execution
if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python embedder.py <source_directory>")
        sys.exit(1)
    
    source_dir = sys.argv[1]
    run_ingestion_pipeline(source_dir)