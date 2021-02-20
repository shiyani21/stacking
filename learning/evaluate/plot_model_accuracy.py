import argparse
import pickle
import numpy as np
import matplotlib.pyplot as plt
import torch

from learning.active.utils import ActiveExperimentLogger
from learning.domains.towers.active_utils import get_sequential_predictions

tower_heights = [2, 3, 4, 5]
min_towers_acq = 40         # number of towers in initial dataset
towers_per_acq = 10         # number of towers acquired between each trained model
                
def calc_model_accuracy(logger, dataset, args, exp_path):
    if args.max_acquisitions is not None: 
        eval_range = range(0, args.max_acquisitions, args.plot_step)
    elif args.single_acquisition_step is not None: 
        eval_range = [args.single_acquisition_step]

    accuracies = {key: [] for key in dataset.keys()}
    for tx in eval_range:
        print('Acquisition step '+str(tx))
        ensemble = logger.get_ensemble(tx)
        preds = get_sequential_predictions(dataset, ensemble)
        preds = preds.mean(axis=1).round()
        
        samples_per_height = int(preds.shape[0]/4) # all preds are in a 1D array
    
        for ti, tower_height in enumerate(tower_heights):
            key = str(tower_height)+'block'
            n_correct = 0
            offset = ti*samples_per_height
            for li, label in enumerate(dataset[key]['labels']):
                if preds[offset+li] == label[0]: n_correct += 1
            accuracies[key].append(n_correct/samples_per_height)
        
    # plot and save to this exp_path
    acquisition_plot_steps = len(range(0, args.max_acquisitions, args.plot_step))
    xs = np.arange(min_towers_acq, \
                    min_towers_acq+towers_per_acq*args.plot_step*acquisition_plot_steps, \
                    towers_per_acq*args.plot_step) # number of training towers
    fig, axes = plt.subplots(4, figsize=(5,12))
    for i, (th, th_accuracies) in enumerate(accuracies.items()):
        axes[i].plot(xs, th_accuracies, label=exp_path)
        axes[i].set_ylim(.5, 1.)
        axes[i].set_ylabel('Constructability Accuracy')
        axes[i].set_xlabel('Training Towers')
        axes[i].set_title(str(th)+' Block Tower Constructability Accuracy')
        axes[i].legend()
    plt.tight_layout()
    plt_fname = 'constructability_accuracy.png'
    plt.savefig(logger.get_figure_path(plt_fname))
    plt.close()
    return accuracies
    
def plot_all_model_accuracies(all_model_accuracies):
    fig, axes = plt.subplots(4, figsize=(5,12))
    
    acquisition_plot_steps = len(range(0, args.max_acquisitions, args.plot_step))
    xs = np.arange(min_towers_acq, \
                    min_towers_acq+towers_per_acq*args.plot_step*acquisition_plot_steps, \
                    towers_per_acq*args.plot_step) # number of training towers

    for pi, th in enumerate(tower_heights):
        for model_accuracies in all_model_accuracies:
            pi_accuracies = model_accuracies[str(th)+'block']
            axes[pi].plot(xs, pi_accuracies)
            axes[pi].set_ylim(.5, 1.)
            axes[pi].set_ylabel('Constructability Accuracy')
            axes[pi].set_xlabel('Training Towers')
            axes[pi].set_title(str(th)+' Block Tower Constructability Accuracy')
        
    plt.tight_layout()
    plt.savefig(args.output_fname)
    plt.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp-paths', 
                        nargs='+', 
                        required=True)
    parser.add_argument('--max-acquisitions',
                        type=int, 
                        help='evaluate from 0 to this acquisition step (use either this or --acquisition-step)')
    parser.add_argument('--plot-step',
                        type=int,
                        default=10)
    parser.add_argument('--single-acquisition-step',
                        type=int,
                        help='evaluate only this acquisition step(use either this or --max-acquisitions)')
    parser.add_argument('--test-set-fname',
                        type=str,
                        required=True,
                        help='evaluate only this acquisition step(use either this or --max-acquisitions)')                        
    parser.add_argument('--output-fname',
                        type=str,
                        required=True,
                        help='evaluate only this acquisition step(use either this or --max-acquisitions)')                        
    parser.add_argument('--debug',
                        action='store_true',
                        help='set to run in debug mode')
    
    args = parser.parse_args()

    if args.debug:
        import pdb; pdb.set_trace()

    with open(args.test_set_fname, 'rb') as f:
        dataset = pickle.load(f)
        
    all_accuracies = []
    for exp_path in args.exp_paths:
        logger = ActiveExperimentLogger(exp_path)
        model_accuracies = calc_model_accuracy(logger, dataset, args, exp_path)
        all_accuracies.append(model_accuracies)
        
    if args.single_acquisition_step is None:
        plot_all_model_accuracies(all_accuracies)
    else:
        print('Accuracy per model: ')
        for i, acc in enumerate(all_accuracies):
            print('    Model '+str(i)+':', acc)