import numpy as np
import torch


def bald(predictions, eps=1e-5):
    """ Get the BALD score for each example.
    :param predictions: (N, K) predictions for N datapoints from K models.
    :return: (N,) The BALD score for each of the datapoints.
    """
    mp_c1 = torch.mean(predictions, dim=1)
    mp_c0 = torch.mean(1 - predictions, dim=1)

    m_ent = -(mp_c1 * torch.log(mp_c1+eps) + mp_c0 * torch.log(mp_c0+eps))

    p_c1 = predictions
    p_c0 = 1 - predictions
    ent_per_model = p_c1 * torch.log(p_c1+eps) + p_c0 * torch.log(p_c0+eps)
    ent = torch.mean(ent_per_model, dim=1)

    bald = m_ent + ent

    return bald

def nodewise_bald(predictions):
    """ Calculate the BALD score for each node of the graph network. Then average 
    across nodes to get the BALD score for the complete tower.
    :param predictions: List of length N where each element is a n x B x K matrix.
    n is minibatch size and N is number of minibatches. B may differ across lists.
    """
    bald_scores = []
    for K in range(1, 5):
        single_size_preds = [p for p in predictions if p.shape[1] == K]
        if len(single_size_preds) > 0:
            single_size_preds = torch.cat(single_size_preds, dim=0)
            N, B, K = single_size_preds.shape
            bald_per_node = bald(single_size_preds.view(N*B, K))
            tower_bald = bald_per_node.view(N, B).mean(dim=1)
            bald_scores.append(tower_bald)

    bald_scores = torch.cat(bald_scores)
    return bald_scores

def choose_acquisition_data(samples, ensemble, n_acquire, strategy, data_pred_fn, data_subset_fn):
    """ Choose data points with the highest acquisition score
    :param samples: (N,2) An array of unlabelled datapoints which to evaluate.
    :param ensemble: A list of models. 
    :param n_acquire: The number of data points to acquire.
    :param strategy: ['random', 'bald'] The objective to use to choose new datapoints.
    :param data_pred_fn: A handler to get predictions specific on the dataset type.
    :prarm data_subset_fn: A handler to select fewer datapoints.
    :return: (n_acquire, 2) - the samples which to label.
    """
    # Get predictions for each model of the ensemble. 
    preds = data_pred_fn(samples, ensemble, ret_nodewise=True)

    # Get the acquisition score for each.
    if strategy == 'bald':
        scores = nodewise_bald(preds).cpu().numpy()
    elif strategy == 'random':
        scores = np.random.uniform(size=preds.shape[0]).astype('float32')
        
    # Return the n_acquire points with the highest score.
    acquire_indices = np.argsort(scores)[::-1][:n_acquire]
    return data_subset_fn(samples, acquire_indices)

def acquire_datapoints(ensemble, n_samples, n_acquire, strategy, data_sampler_fn, data_label_fn, data_pred_fn, data_subset_fn):
    """ Get new datapoints given the current ensemble.
    Uses function handlers for domain specific components (e.g., sampling unlabeled data).
    :param n_samples: How many unlabeled samples to generate.
    :param n_acquire: How many samples to acquire labels for.
    :param strategy: Which acquisition function to use.
    :param data_sampler_fn: Function handler: n_samples -> Dataset
    :param data_label_fn:
    :param data_pred_fn:
    :param data_subset_fn:
    :return: (n_acquire, 2), (n_acquire,) - x,y tuples of the new datapoints.
    """
    unlabeled_pool = data_sampler_fn(n_samples)
    xs = choose_acquisition_data(unlabeled_pool, ensemble, n_acquire, strategy, data_pred_fn, data_subset_fn)
    new_data = data_label_fn(xs)
    return new_data, unlabeled_pool
