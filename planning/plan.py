from collections import namedtuple
import argparse
import pickle
import sys
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from planning.tree import Tree
from planning.problems import Tallest, Overhang, Deconstruct
from block_utils import Object
from learning.active.utils import ActiveExperimentLogger

def plan(timeout, blocks, problem, model):
    tree = Tree(blocks)
    for t in range(timeout):
        parent_node_id = tree.get_exp_best_node_expand()
        #print(t, len(tree.nodes[parent_node_id]['tower']), tree.nodes[parent_node_id]['value'])
        sys.stdout.write("Search progress: %i   \r" % (t) )
        sys.stdout.flush()
        new_nodes = problem.sample_actions(tree.nodes[parent_node_id], model)
        for node in new_nodes:
            tree.expand(parent_node_id, node)
    return tree

def plan_mcts(logger, timeout, blocks, problem, model, c=1., discrete=True):
    tree = Tree(blocks)
    tallest_tower = [0]
    highest_exp_height = [0]
    highest_value = [0]
    tower_stats = np.zeros((problem.max_height,timeout))
    node_values = {k:{'median':[],'25':[],'75':[]} for k in range(problem.max_height+1)}
    
    for t in range(timeout):
        tower_stats[:,t] = tower_stats[:,t-1]
        sys.stdout.write("Search progress: %i   \r" % (t) )
        sys.stdout.flush()
        parent_node_id = tree.traverse(c)
        
        new_nodes = problem.sample_actions(tree.nodes[parent_node_id], model, discrete=discrete)
        tallest_tower_t = tallest_tower[-1]
        highest_exp_height_t = highest_exp_height[-1]
        highest_value_t = highest_value[-1]
        for new_node in new_nodes:
            #print(t, len(new_node['tower']), new_node['exp_reward'])
            new_node_id = tree.expand(parent_node_id, new_node)
            rollout_value = tree.rollout(new_node_id, problem, model)
            tree.backpropagate(new_node_id, rollout_value)
            
            tower_height = len(new_node['tower'])
            #print(tower_height)
            index = int(tower_height)
            tower_stats[index-1,t] += 1
            if len(new_node['tower'])>tallest_tower_t:
                tallest_tower_t = len(new_node['tower'])
                
            if new_node['exp_reward'] > highest_exp_height_t:
                highest_exp_height_t = new_node['exp_reward']
                
            if new_node['value'] > highest_value_t:
                highest_value_t = new_node['value']
                
        tallest_tower.append(tallest_tower_t)
        highest_exp_height.append(highest_exp_height_t)
        highest_value.append(highest_value_t)
        
        # update node value stats
        temp_values = {k:[] for k in range(problem.max_height+1)}
        for node in tree.nodes:
            height = len(tree.nodes[node]['tower']) 
            temp_values[height].append(tree.nodes[node]['value'])
        for height in range(problem.max_height+1):
            if temp_values[height] == []:
                node_values[height]['median'].append(0)
                node_values[height]['25'].append(0)
                node_values[height]['75'].append(0)
            else:
                node_values[height]['median'].append(np.median(temp_values[height]))
                node_values[height]['25'].append(np.quantile(temp_values[height], .25))
                node_values[height]['75'].append(np.quantile(temp_values[height], .75))
        
    return tree, tallest_tower, highest_exp_height, highest_value, tower_stats, node_values
    
def plot_histogram(logger, tower_stats, c):
    plt.figure()
    xs = list(range(tower_stats.shape[1]))
    keys = ['2block', '3block', '4block', '5block']
    plt.bar(xs, tower_stats[0,:], label='1block')
    for i in range(0, tower_stats.shape[0]-1):
        plt.bar(xs, tower_stats[i+1,:], bottom=np.sum(tower_stats[:i+1,:], axis=0), label=keys[i])
    plt.title('c= '+str(c))
    plt.legend()
    timestamp = datetime.now().strftime("%d-%m-%H-%M-%S")
    fig_title = 'mcts_test_hist_'+timestamp+'.png'
    plt.savefig(logger.get_figure_path(fig_title))
    plt.close()
    
def plot_run_data(logger, all_tallest_towers, all_highest_exp_heights, all_highest_values, c_vals):
    median_tt = np.median(all_tallest_towers, axis=2)
    median_hev = np.median(all_highest_exp_heights, axis=2)
    median_hv = np.median(all_highest_values, axis=2)
    
    q25_tt = np.quantile(all_tallest_towers, 0.25, axis=2)
    q75_tt = np.quantile(all_tallest_towers, 0.75, axis=2)
    
    q25_hev = np.quantile(all_highest_exp_heights, 0.25, axis=2)
    q75_hev = np.quantile(all_highest_exp_heights, 0.75, axis=2)
    
    q25_hv = np.quantile(all_highest_values, 0.25, axis=2)
    q75_hv = np.quantile(all_highest_values, 0.75, axis=2)
    
    xs = list(range(len(all_tallest_towers[:,0,0])))
    all_plot_data = [(median_tt, q25_tt, q75_tt),
                        (median_hev, q25_hev, q75_hev),
                        (median_hv, q25_hv, q75_hv)]
    for pi, (data, title) in enumerate(zip(all_plot_data,
                    ['tallest_tower', 'highest_expected_value', 'highest_UCT_value'])):
        fig, ax = plt.subplots()
        for ci, c in enumerate(c_vals):
            ax.plot(xs, data[0][:,ci], label='c='+str(c))
            ax.fill_between(xs, data[1][:,ci], data[2][:,ci], alpha=0.2)
        
        ax.legend()
        ax.set_ylabel(title)
        ax.set_xlabel('time')
        ax.set_title(title)
        #ax.set_yscale('log')
        
        timestamp = datetime.now().strftime("%d-%m-%H-%M-%S")
        fig_title = 'mcts_test_'+title+'_'+str(timestamp)+'.png'

        plt.savefig(logger.get_figure_path(fig_title))
        plt.close()
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--problem', 
                        choices=['tallest', 'overhang', 'deconstruct'], 
                        default='tallest',
                        help='planning problem/task to plan for')
    parser.add_argument('--block-set-fname', 
                        type=str, 
                        default='',
                        help='path to the block set file. if not set, random 5 blocks generated.')
    parser.add_argument('--exp-path', 
                        type=str, 
                        required=True)
    parser.add_argument('--timeout',
                        type=int,
                        default=1000,
                        help='max number of iterations to run planner for')
    parser.add_argument('--debug',
                        action='store_true',
                        help='set to run in debug mode')
    parser.add_argument('--max-height',
                        type=int,
                        default=5,
                        help='number of blocks in desired tower')
    parser.add_argument('--discrete',
                        action='store_true')
    parser.add_argument('--n-blocks',
                        type=int)
    args = parser.parse_args()
    
    if args.debug:
        import pdb; pdb.set_trace()
    
    tx = 99
    
    if args.block_set_fname is not '':
        with open(args.block_set_fname, 'rb') as f:
            block_set = pickle.load(f)
    else:
        try:
            args.n_blocks
        except:
            print('if no block set is provided, but give the number of blocks \
                    to generate in random set')
        block_set = [Object.random(f'obj_{ix}') for ix in range(n_blocks)]
        
    if args.problem == 'tallest':
        problem = Tallest(args.max_height)
    elif args.problem == 'overhang':
        problem = Overhang()
    elif args.problem == 'deconstruct':
        problem = Deconstruct()
        
    logger = ActiveExperimentLogger(args.exp_path)
    ensemble = logger.get_ensemble(tx)
    
    c_vals = [1.0]#[0, 0.1, 0.2, 0.5, 1., np.sqrt(2), 7, 10, 15]
    num_c_vals = len(c_vals)
    runs = 1
    
    all_tallest_towers = np.zeros((args.timeout+1, num_c_vals, runs))
    all_highest_exp_heights = np.zeros((args.timeout+1, num_c_vals, runs))
    all_highest_values = np.zeros((args.timeout+1, num_c_vals, runs))
    
    for ci, c in enumerate(c_vals):
        for ri in range(runs):
            tree, tallest_tower, highest_exp_height, highest_value, tower_stats, node_values = \
                plan_mcts(logger, args.timeout, block_set, problem, ensemble, c=c, discrete=args.discrete)
                
            ## Plot histogram of tower heights
            plot_histogram(logger, tower_stats, c)
            
            all_tallest_towers[:,ci, ri] = tallest_tower
            all_highest_exp_heights[:,ci, ri] = highest_exp_height
            all_highest_values[:,ci, ri] = highest_value
            
    plot_run_data(logger, all_tallest_towers, all_highest_exp_heights, all_highest_values, c_vals)