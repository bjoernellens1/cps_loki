import os
import subprocess
import yaml

class GitCloner:
    def __init__(self, directory, repos_file):
        self.directory = directory
        self.maindir = os.path.dirname(os.path.abspath(__file__)) # should be equal to the current directory where the script is called from
        self.repos_file = repos_file

    def clone_or_update_repos(self):
        # Load repository URLs from YAML file
        with open(self.repos_file, 'r') as file:
            repos_data = yaml.safe_load(file)

        # Clone or update repositories
        for repo_data in repos_data['repositories']:
            repo_url = repo_data['url']
            branch = repo_data.get('branch', 'main')
            self.clone_or_update_repo(repo_url, branch)

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

    def build_repos(self):
        try:
            # Change working directory to the base directory
            main_dir = os.path.join(self.maindir)
            os.chdir(main_dir)

            # Execute colcon build
            subprocess.check_output(['colcon', 'build'])

            print("Build completed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to build repositories: {e}")

    def source_setup(self):
        try:
            # Change working directory to the base directory
            main_dir = os.path.join(self.maindir)
            os.chdir(main_dir)

            # Execute "source install/setup.bash" from the parent directory
            subprocess.check_call(['source', os.path.join('install', 'setup.bash')], shell=True)

            print("Setup file sourced successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to source setup file: {e}")


# Usage example
cloner = GitCloner('./src', 'repos.yaml')
cloner.clone_or_update_repos()
cloner.build_repos()
cloner.source_setup()

