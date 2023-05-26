import os
import subprocess
import yaml

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
            print(f"Successfully cloned repository '{repo_name}' on branch '{branch}'.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to clone repository '{repo_name}' on branch '{branch}': {e}")


# Usage example
cloner = GitCloner('./src')

# Load repository URLs from YAML file
with open('repos.yaml', 'r') as file:
    repos_data = yaml.safe_load(file)

# Clone repositories
for repo_data in repos_data['repositories']:
    repo_url = repo_data['url']
    branch = repo_data.get('branch', 'main')
    cloner.clone_repo(repo_url, branch)
