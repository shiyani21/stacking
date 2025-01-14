import argparse
import pickle
import numpy as np

from learning.active.utils import ActiveExperimentLogger
from learning.domains.towers.generate_tower_training_data import sample_random_tower
from tower_planner import TowerPlanner
from block_utils import Object, get_rotated_block

# make datasets for all tower heights with 50/50 constructable not constructable split
# from given block_set
def make_test_dataset(blocks, args):
    dataset = {}
    num_blocks = [2, 3, 4, 5]
    tp = TowerPlanner()
    
    for k in num_blocks:
        print('Generating '+str(k)+' block towers...')
        key = str(k)+'block'
        towers = []
        labels = [] # each tower is labeled with (constructable, stable, pw_stable, cog_stable)
        done = False
        n_constructable = 0
        while True:
            if not blocks:
                tower_blocks = [Object.random() for _ in range(k)]
            else: 
                tower_blocks = np.random.choice(blocks, k)
            tower = sample_random_tower(tower_blocks, num_blocks=k)
            rotated_tower = [get_rotated_block(b) for b in tower]
            vec_tower = [block.vectorize() for block in rotated_tower]
            
            constructable = tp.tower_is_constructable(rotated_tower)
            stable = tp.tower_is_stable(rotated_tower)
            pw_stable = tp.tower_is_pairwise_stable(rotated_tower)
            cog_stable = tp.tower_is_cog_stable(rotated_tower)
            label = (constructable, stable, pw_stable, cog_stable)
            if constructable and (n_constructable < args.samples_per_height/2): 
                n_constructable += 1
                labels.append(label)    
                towers.append(vec_tower)
            elif (not constructable) and len(towers)-n_constructable < args.samples_per_height/2:
                labels.append(label)
                towers.append(vec_tower)
            elif len(towers) == args.samples_per_height:
                break
        
        dataset[key] = {'towers': np.array(towers), 'labels': labels}
        
    with open(args.output_fname, 'wb') as f:
        pickle.dump(dataset, f)
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--block-set-fname', 
                        type=str, 
                        default='',
                        help='path to the block set file. if not set, args.n_blocks random blocks generated.')
    parser.add_argument('--samples-per-height',
                        type=int,
                        default=1000)
    parser.add_argument('--output-fname',
                        type=str,
                        required=True)
    parser.add_argument('--debug',
                        action='store_true',
                        help='set to run in debug mode')
    
    args = parser.parse_args()

    if args.debug:
        import pdb; pdb.set_trace()
 
    if args.block_set_fname != '':
        with open(args.block_set_fname, 'rb') as f:
            block_set = pickle.load(f)
    else:
        block_set = None
        
    make_test_dataset(block_set, args)
    