import os
import re
from pathlib import Path
from typing import List, Dict, Any
import logging


logger = logging.getLogger(__name__)


class ContentParser:
    def __init__(self):
        self.supported_extensions = ['.md', '.mdx']
    
    def parse_directory(self, directory_path: str) -> List[Dict[str, Any]]:
        """
        Parse all MD/MDX files in a directory and extract content with metadata.
        
        Args:
            directory_path: Path to the directory containing MD/MDX files
            
        Returns:
            List of dictionaries containing file content and metadata
        """
        all_content = []
        
        for root, dirs, files in os.walk(directory_path):
            for file in files:
                if any(file.endswith(ext) for ext in self.supported_extensions):
                    file_path = os.path.join(root, file)
                    try:
                        file_content = self.parse_file(file_path)
                        
                        # Create a relative path from the docs directory
                        relative_path = os.path.relpath(file_path, directory_path)
                        # Convert to a URL-style path
                        source_page = relative_path.replace('\\', '/').replace('.md', '').replace('.mdx', '')
                        
                        all_content.append({
                            'source_page': f"/{source_page}",
                            'content': file_content,
                            'section_title': self._extract_title(file_content) or os.path.basename(file_path),
                            'metadata': {
                                'file_path': file_path,
                                'relative_path': relative_path,
                                'file_size': os.path.getsize(file_path)
                            }
                        })
                    except Exception as e:
                        logger.error(f"Error parsing file {file_path}: {e}")
        
        return all_content
    
    def parse_file(self, file_path: str) -> str:
        """
        Parse a single MD/MDX file and extract content.
        
        Args:
            file_path: Path to the file to parse
            
        Returns:
            Extracted content as a string
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Remove frontmatter if it exists
        content = self._remove_frontmatter(content)
        
        # Clean up content (remove special MDX syntax if needed)
        content = self._clean_content(content)
        
        return content
    
    def _remove_frontmatter(self, content: str) -> str:
        """
        Remove frontmatter from content (text between --- delimiters at the start of the file).
        
        Args:
            content: Content that may contain frontmatter
            
        Returns:
            Content without frontmatter
        """
        if content.startswith('---'):
            try:
                # Find the second occurrence of '---'
                end_index = content.find('---', 3)
                if end_index != -1:
                    # Return content after the second '---' plus a newline
                    return content[end_index + 3:].strip()
            except:
                # If anything goes wrong, return the original content
                pass
        
        return content
    
    def _clean_content(self, content: str) -> str:
        """
        Clean content by removing MDX-specific syntax that might interfere with processing.
        
        Args:
            content: Raw content from MD/MDX file
            
        Returns:
            Cleaned content
        """
        # Remove import statements (common in MDX)
        content = re.sub(r'^\s*import\s+.*?\n', '', content, flags=re.MULTILINE)
        
        # Remove export statements
        content = re.sub(r'^\s*export\s+.*?\n', '', content, flags=re.MULTILINE)
        
        # Remove JSX components (simplified approach)
        # This finds code blocks that start with < and end with >
        # content = re.sub(r'<.*?>', '', content, flags=re.DOTALL)
        
        # Remove code block markers but keep the content
        # content = re.sub(r'```.*?\n(.*?)```', r'\1', content, flags=re.DOTALL)
        
        return content.strip()
    
    def _extract_title(self, content: str) -> str:
        """
        Extract the first H1 title from content.
        
        Args:
            content: Content to extract title from
            
        Returns:
            Extracted title or None if not found
        """
        # Look for H1 header
        h1_pattern = r'^#\s+(.*)$'
        match = re.search(h1_pattern, content, re.MULTILINE)
        
        if match:
            return match.group(1).strip()
        
        return None