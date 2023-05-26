import os
import subprocess

class GitCloner:
    def __init__(self, directory):
        self.directory = directory

    def clone_repo(self, repo_url, branch='main'):
        repo_name = repo_url.split('/')[-1].split('.')[0]
        repo_path = os.path.join(self.directory, repo_name)

        if os.path.exists(repo_path):
            print(f"Repository '{repo_name}' already exists. Skipping cloning.")
            return

        command = ['git', 'clone', '--branch', branch, repo_url, repo_path]
        try:
            subprocess.check_output(command)
            print(f"Successfully cloned repository '{repo_name}'.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to clone repository '{repo_name}': {e}")

# Usage example
cloner = GitCloner('./src')

# Load repository URLs from file
with open('repos.txt', 'r') as file:
    repos = file.readlines()

# Clone repositories
for repo_url in repos:
    repo_url = repo_url.strip()  # Remove newline characters
    cloner.clone_repo(repo_url)
