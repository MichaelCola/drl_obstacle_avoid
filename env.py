import config
import vrepinter
import numpy as np

class env():
    def __init__(self):
        self.dis=0
        self.state=None
        self.next_state=None
        self.done=False
        self.reward=0
    
    def start(self,i):
        if i == 0:
            vrepinter.connect()
        vrepinter.start()
        state,resolution = vrepinter.fetch_kinect() #state is kinect see
        print(state.shape)
        print(resolution)
        state = np.tile(state, 4)
        self.state = np.reshape(state, [16384]) # np reshape state
        return state



    def step(self,action):
        v_left = config.valid_actions_dict[action][0]
        v_right = config.valid_actions_dict[action][1]
        next_state = vrepinter.move_wheels(v_left, v_right)
        self.next_state = np.reshape(next_state, [16384])
        self.reward, self.done = get_reward()
        return self.next_state, self.reward, self.done


    def get_reward(self):
        done = False
        collision = vrepinter.if_collision()
        dis=vrepinter.get_dis()
        if collision == 1:
            done = True
            reward = -500
        reward = 10*(dis-self.dis)-1
        self.dis=dis
        return reward, done
    
    def reset(self):
        vrepinter.stop()
        self.dis=0
        self.state=None
        self.next_state=None
        self.done=False
        self.reward=0
    
def main():
    envrionment=env()
    envrionment.start(0)
    envrionment.reset()
    
if __name__ == '__main__':
    main()





