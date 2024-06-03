import json
import requests
import yaml
from langchain.agents import tool

# Global variables for DokuWiki API access
API_URL = "http://127.0.0.1:9000/jsonrpc"
HEADERS = {'Content-Type': 'application/json'}

@tool
def list_pages(namespace: str, depth: int) -> str:
    """
    List pages in the given namespace up to a certain depth in the DokuWiki.
    Args:
    - namespace (str): The namespace to search within.
    - depth (int): How deep to search within the namespace.
    
    Returns:
    - str: YAML string of the list of pages.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.listPages",
        "params": [namespace, depth],
        "id": 1
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def search_pages(query: str) -> str:
    """
    Search for pages containing the specified query text.
    Args:
    - query (str): Text query to search for within page contents.
    
    Returns:
    - str: YAML string of the search results.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.search",
        "params": [query],
        "id": 2
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def get_page(page: str) -> str:
    """
    Retrieve the content of a specific page.
    Args:
    - page (str): The ID of the page to retrieve.
    
    Returns:
    - str: YAML string of the page content.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.getPage",
        "params": [page],
        "id": 3
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

# Additional functions can be implemented in a similar way
# Here is an example for saving a page

@tool
def save_page(page: str, text: str, summary: str, is_minor: bool) -> str:
    """
    Save or update the content of a page.
    Args:
    - page (str): The ID of the page to save or update.
    - text (str): The content to write to the page.
    - summary (str): Summary of the changes made.
    - is_minor (bool): Flag to indicate if the change is minor.
    
    Returns:
    - str: YAML string of the server's response.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.putPage",
        "params": [page, text, summary, is_minor],
        "id": 4
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def get_recent_page_changes(timestamp: int) -> str:
    """
    Get recent changes in the wiki pages since a given timestamp.
    Args:
    - timestamp (int): UNIX timestamp to fetch changes since.
    
    Returns:
    - str: YAML string of the list of recent changes.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.getRecentChanges",
        "params": [timestamp],
        "id": 5
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def get_page_info(page: str) -> str:
    """
    Retrieve metadata about a specific page.
    Args:
    - page (str): The ID of the page to retrieve info for.
    
    Returns:
    - str: YAML string of the page metadata.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.getPageInfo",
        "params": [page],
        "m_id": 6
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def get_page_history(page: str) -> str:
    """
    Get the history of edits for a specified page.
    Args:
    - page (str): The ID of the page whose history is to be retrieved.
    
    Returns:
    - str: YAML string of the edit history.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.getPageHistory",
        "params": [page],
        "id": 7
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.filename)

@tool
def append_page(page: str, text: str, summary: str, is_minor: bool) -> str:
    """
    Append text to an existing page.
    Args:
    - page (str): The ID of the page to append to.
    - text (str): Text to append to the page.
    - summary (str): Summary of the changes.
    - is_minor (bool): Whether the edit is minor.
    
    Returns:
    - str: YAML string of the server's response.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.appendPage",
        "params": [page, text, summary, is_minor],
        "id": 8
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def get_page_links(page: str) -> str:
    """
    Retrieve a list of links present on a specified page.
    Args:
    - page (str): The ID of the page to retrieve links from.
    
    Returns:
    - str: YAML string of the links found on the page.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.listLinks",
        "params": [page],
        "id": 9
    }
    response = requests.post(API_API, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def get_page_backlinks(page: str) -> str:
    """
    Retrieve a list of all pages that link back to a specified page.
    Args:
    - page (str): The ID of the page to check for backlinks.
    
    Returns:
    - str: YAML string of pages linking back to the specified page.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.getBackLinks",
        "params": [page],
        "id": 10
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def list_media(namespace: str="", pattern: str="", depth: int=1) -> str:
    """
    List media files in a given namespace that match a pattern.
    Args:
    - namespace (str): Namespace to search within.
    - pattern (str): Pattern to filter the media files.
    - depth (int): Search depth within the namespace.
    
    Returns:
    - str: YAML string of the list of media files.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.listMedia",
        "params": [namespace, pattern, depth],
        "id": 11
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def get_recent_media_changes(timestamp: int) -> str:
    """
    Get recent changes in the media files since a given timestamp.
    Args:
    - timestamp (int): UNIX timestamp to fetch changes since.
    
    Returns:
    - str: YAML string of the list of recent changes in media files.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.getRecentMediaChanges",
        "params": [timestamp],
        "id": 12
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def get_media(media: str, rev: int) -> str:
    """
    Retrieve a specific revision of a media file.
    Args:
    - media (str): The ID of the media file to retrieve.
    - rev (int): The revision number of the media file.
    
    Returns:
    - str: YAML string of the media file data.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.getMedia",
        "params": [media, rev],
        "id": 13
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def get_media_info(media: str) -> str:
    """
    Retrieve metadata about a specific media file.
    Args:
    - media (str): The ID of the media file to retrieve info for.
    
    Returns:
    - str: YAML string of the media metadata.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.getMediaInfo",
        "params": [media],
        "id": 14
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe.dump(response.json())

@tool
def save_media(media: str, base64: str, overwrite: bool) -> str:
    """
    Save or update a media file encoded in base64 format.
    Args:
    - media (str): The ID of the media file to save or update.
    - base64 (str): Base64 encoded data of the file.
    - overwrite (bool): Whether to overwrite the existing file.
    
    Returns:
    - str: YAML string of the server's response.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.putMedia",
        "params": [media,base64, overwrite],
        "id": 15
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe_dump(response.json())

@tool
def delete_media(media: str) -> str:
    """
    Delete a specific media file.
    Args:
    - media (str): The ID of the media file to be deleted.
    
    Returns:
    - str: YAML string of the server's response after attempting to delete the media file.
    """
    payload = {
        "jsonrpc": "2.0",
        "method": "dokuwiki.deleteMedia",
        "params": [media],
        "id": 16
    }
    response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
    return yaml.safe.dump(response.json())

wiki_tools = [
    list_pages,
    search_pages,
    get_page,
    get_recent_page_changes,
    get_page_info,
    get_page_history,
    get_page_links,
    get_page_backlinks,
    save_page,
    append_page,
    list_media,
    get_recent_media_changes,
    get_media,
    get_media_info,
    save_media,
    delete_media
]