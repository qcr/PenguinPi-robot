from locust import HttpLocust, TaskSet, task, between

class UserBehaviour(TaskSet):
    def on_start(self):
        """ on_start is called when a Locust start before any task is scheduled """
        #self.login()

    def on_stop(self):
        """ on_stop is called when the TaskSet is stopping """
        #self.logout()


    #@task(2)
    #def index(self):
    #    self.client.get("/camera/get")

    @task(1)
    def get_pose(self):
        self.client.get("/pose/get")

class WebsiteUser(HttpLocust):
    task_set = UserBehaviour

    # Define how long the user waits in between requests
    wait_time = between(0.25, 0.27)
