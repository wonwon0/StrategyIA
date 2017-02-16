# Under MIT License, see LICENSE.txt


class Executor(object):
    """ Classe abstraite des executeurs. """

    def __init__(self, p_world_state):
        self.ws = p_world_state

    def exec(self):
        """ Méthode qui sera appelé à chaque coup de boucle. """
        pass
