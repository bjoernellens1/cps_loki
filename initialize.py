import os
import subprocess
import yaml

class GitCloner:
    def __init__(self, directory):
        self.directory = directory
        self.maindir = os.path.dirname(os.path.abspath(__file__)) # should be equal to the current directory where the script is called from

    def clone_or_update_repo(self, repo_url, branch='main'):
        repo_name = repo_url.split('/')[-1].split('.')[0]
        repo_path = os.path.join(self.directory, repo_name)

        if os.path.exists(repo_path):
            print(f"Repository '{repo_name}' already exists. Updating...")
            try:
                # Change working directory to the repository path
                os.chdir(repo_path)

                # Pull the latest changes from the repository
                subprocess.check_output(['git', 'pull'])

                print(f"Successfully updated repository '{repo_name}'.")
            except subprocess.CalledProcessError as e:
                print(f"Failed to update repository '{repo_name}': {e}")
                return
        else:
            try:
                # Clone the repository
                subprocess.check_output(['git', 'clone', '--branch', branch, repo_url, repo_path])

                print(f"Successfully cloned repository '{repo_name}' on branch '{branch}'.")
            except subprocess.CalledProcessError as e:
                print(f"Failed to clone repository '{repo_name}' on branch '{branch}': {e}")
                return

        # Build the repository using colcon
        try:
            # Change working directory to the base directory
            main_dir = os.path.join(self.maindir)
            os.chdir(maindir)

            # Execute colcon build
            subprocess.check_output(['colcon', 'build'])

            print("Build completed successfully.")

            # Execute "source install/setup.bash"
            subprocess.check_call(['source', 'install/setup.bash'], shell=True)

            print("Setup file sourced successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to build or source setup file: {e}")


# Usage example
cloner = GitCloner('./src')

# Load repository URLs from YAML file
with open('repos.yaml', 'r') as file:
    repos_data = yaml.safe_load(file)

# Clone or update repositories and perform build and setup
for repo_data in repos_data['repositories']:
    repo_url = repo_data['url']
    branch = repo_data.get('branch', 'main')
    cloner.clone_or_update_repo(repo_url, branch)
