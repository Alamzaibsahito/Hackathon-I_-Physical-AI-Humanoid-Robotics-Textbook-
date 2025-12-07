import re

def clean_markdown(text: str) -> str:
    """Cleans markdown text by removing extra spaces, newlines, and specific markdown syntax."""
    # Remove frontmatter (assuming it's at the very beginning of the file)
    text = re.sub(r'^---\n(.*?\n)+?---\n', '', text, flags=re.DOTALL)
    # Remove HTML comments
    text = re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)
    # Replace multiple newlines with a single space
    text = re.sub(r'\n\s*\n', ' ', text)
    # Replace single newlines with a space
    text = text.replace('\n', ' ')
    # Remove extra spaces
    text = re.sub(r'\s+', ' ', text).strip()
    return text
