import os
from typing import Optional, Dict

# Placeholder for better-auth.com SDK/API interactions
# In a real scenario, this would have actual API calls to better-auth.com

BETTER_AUTH_API_URL = os.getenv("BETTER_AUTH_API_URL", "https://api.better-auth.com")

async def register_user(username: str, email: str, password: str, background_info: Optional[Dict] = None) -> Dict:
    print(f"[better_auth_service] Simulating user registration for {email}")
    # Simulate API call to better-auth.com
    # Example response structure
    return {"user_id": "mock_user_123", "email": email, "message": "Registration simulated successfully"}

async def login_user(email: str, password: str) -> Dict:
    print(f"[better_auth_service] Simulating user login for {email}")
    # Simulate API call to better-auth.com
    return {"user_id": "mock_user_123", "email": email, "token": "mock_jwt_token", "message": "Login simulated successfully"}

async def get_user_profile(token: str) -> Optional[Dict]:
    print(f"[better_auth_service] Simulating fetching user profile with token: {token[:10]}...")
    # Simulate API call to better-auth.com
    if token:
        return {"user_id": "mock_user_123", "email": "user@example.com", "software_experience": "Python", "hardware_experience": "ROS"}
    return None

async def update_user_background(user_id: str, background_info: Dict, token: str) -> Dict:
    print(f"[better_auth_service] Simulating updating background for {user_id}: {background_info}")
    # Simulate API call to better-auth.com to update user metadata
    return {"user_id": user_id, "message": "Background info updated successfully"}
