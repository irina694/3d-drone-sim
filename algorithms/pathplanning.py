"""
This file runs before the simulation starts in Unreal Engine 5.
The file gets data from the Unreal Editor about the scene and the drone,
and outputs a path plan (path steps) for the drone to fly in a given 3D scene.
The output is saved to a JSON file, which is then loaded and processed by the game engine during the simulation.

@author: Irina Hallinan <irina_hallinan@berkeley.edu>
"""


import unreal

print("Loaded path planning script...")
drone_asset = unreal.EditorAssetLibrary(name='Drone_BP')
print(drone_asset)


actorsList = unreal.EditorLevelLibrary.get_all_level_actors()
for actor in actorsList:
    actorLabel = actor.get_actor_label()
    actorPos = actor.get_actor_location()

    #print('actorLabel= %s actorPos=%s' % (actorLabel, actorPos))

    if actorLabel == 'palace_of_fine_arts_scaled':
        print('Found the scene!')
        print('actorLabel= %s actorPos=%s' % (actorLabel, actorPos))

    if actorLabel == 'Drone_BP':
        print('Found the drone!')
        print('actorLabel= %s actorPos=%s' % (actorLabel, actorPos))