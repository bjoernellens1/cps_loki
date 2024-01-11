import requests
import json

url = "https://loki.local:9443/api/auth"

# Set your admin username and password
username = "cpsadmin"
password = "pac99CPS"

# Define the payload for the POST request
payload = {
    "Username": username,
    "Password": password
}

headers = {
    "Content-Type": "application/json"
}

response = requests.post(url, data=json.dumps(payload), headers=headers, verify=False)


# Check the response status code
if response.status_code == 200:
    print("Authentication successful!")
    # You can access the response content using response.text or response.json() if the content is in JSON format
    print("Response content:", response.text)
else:
    print("Authentication failed. Status code:", response.status_code)
    print("Response content:", response.text)

token = response
