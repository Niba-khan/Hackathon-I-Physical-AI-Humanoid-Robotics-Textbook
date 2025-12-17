import re
from typing import List, Dict, Any
from src.utils import logger


class TextChunker:
    def __init__(self, chunk_size: int = 1000, overlap_size: int = 200):
        """
        Initialize the text chunker.
        
        Args:
            chunk_size: Maximum size of each chunk
            overlap_size: Size of overlap between chunks
        """
        self.chunk_size = chunk_size
        self.overlap_size = overlap_size
    
    def chunk_text(self, text: str, source_page: str, section_title: str, 
                   max_chunk_size: int = 10000) -> List[Dict[str, Any]]:
        """
        Split text into overlapping chunks.
        
        Args:
            text: The text to chunk
            source_page: The source page identifier
            section_title: The section title
            max_chunk_size: Maximum size for a chunk (defaults to 10000 chars)
            
        Returns:
            List of chunk dictionaries with metadata
        """
        chunks = []
        text_length = len(text)
        start_idx = 0
        
        chunk_id = 1
        while start_idx < text_length:
            # Determine the end index for this chunk
            end_idx = start_idx + min(max_chunk_size, text_length - start_idx)
            
            # Get the chunk text
            chunk_text = text[start_idx:end_idx]
            
            # Add chunk to the list
            chunk = {
                'chunk_id': f"{source_page.replace('/', '_')}_{chunk_id}",
                'source_page': source_page,
                'section_title': section_title,
                'content': chunk_text.strip(),
                'metadata': {
                    'start_idx': start_idx,
                    'end_idx': end_idx,
                    'chunk_order': chunk_id
                }
            }
            
            chunks.append(chunk)
            chunk_id += 1
            
            # Move start index for next chunk (with overlap)
            start_idx = end_idx - self.overlap_size if self.overlap_size < end_idx else end_idx
            
            # If the overlap would create a chunk that's too small, skip to the end
            if end_idx == text_length:
                break
        
        logger.info(f"Created {len(chunks)} chunks from {source_page}")
        return chunks
    
    def chunk_by_sentence(self, text: str, source_page: str, section_title: str) -> List[Dict[str, Any]]:
        """
        Split text into chunks by sentences, respecting the maximum chunk size.
        
        Args:
            text: The text to chunk
            source_page: The source page identifier
            section_title: The section title
            
        Returns:
            List of chunk dictionaries with metadata
        """
        # Split text into sentences using regex
        sentences = re.split(r'(?<=[.!?]) +', text)
        
        chunks = []
        current_chunk = ""
        current_length = 0
        chunk_id = 1
        
        for sentence in sentences:
            # If adding this sentence would exceed the chunk size
            if current_length + len(sentence) > self.chunk_size and current_chunk:
                # Finalize the current chunk and start a new one
                chunk = {
                    'chunk_id': f"{source_page.replace('/', '_')}_s{chunk_id}",
                    'source_page': source_page,
                    'section_title': section_title,
                    'content': current_chunk.strip(),
                    'metadata': {
                        'chunk_type': 'sentence-based',
                        'chunk_order': chunk_id
                    }
                }
                chunks.append(chunk)
                
                # Start a new chunk with the current sentence
                current_chunk = sentence + " "
                current_length = len(current_chunk)
                chunk_id += 1
            else:
                # Add the sentence to the current chunk
                current_chunk += sentence + " "
                current_length += len(sentence) + 1
        
        # Add the final chunk if there's content
        if current_chunk.strip():
            chunk = {
                'chunk_id': f"{source_page.replace('/', '_')}_s{chunk_id}",
                'source_page': source_page,
                'section_title': section_title,
                'content': current_chunk.strip(),
                'metadata': {
                    'chunk_type': 'sentence-based',
                    'chunk_order': chunk_id
                }
            }
            chunks.append(chunk)
        
        logger.info(f"Created {len(chunks)} sentence-based chunks from {source_page}")
        return chunks