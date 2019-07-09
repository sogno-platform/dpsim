import csv
import pandas
import numpy

import matplotlib.pyplot as plt

class Measurement:
    """ Generalized set of measurements (usually wall clock times per timestep)
        generated with the same, otherwise fixed settings.
    """

    def __init__(self, xaxis, xlabel, data, ylabel, name=None, instance=None, sigma=None):
        self.instance = instance
        self.xaxis = xaxis
        self.xlabel = xlabel
        self.data = data
        self.ylabel = ylabel

        if sigma:
            self.sigma = sigma
        else:
            self.sigma = {}

        if name:
            self.name = name
        elif self.instance:
            self.name = self.instance.name
        else:
            self.name = ''

    @staticmethod
    def read_csv(filename):
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            xlabel = reader.fieldnames[0]
            data = {}
            sigma = {}
            xaxis = []
            for name in reader.fieldnames[1:]:
                if not name.startswith('sigma '):
                    data[name] = []
                    sigma[name] = []
            for row in reader:
                for k, v in row.items():
                    if k == xlabel:
                        xaxis.append(int(v))
                    elif k.startswith('sigma '):
                        sigma[k[6:]].append(float(v))
                    else:
                        data[k].append(float(v))
            return Measurement(None, xaxis, xlabel, data, filename.rstrip('.csv'), sigma)

    @staticmethod
    def read_timestep_csv(filename):       
        pd_df = pandas.read_csv(filename)
        pd_df.rename(columns=lambda x: x.strip(), inplace=True)
        column_names = list(pd_df.columns.values)

        measurements = {}
        for column in column_names:
            data = pd_df[column].values
            xaxis = numpy.arange(data.shape[0])
            xlabel = 'timestep'
            ylabel = column    
            measurements[column] = Measurement(xaxis, xlabel, data, ylabel)
        
        return measurements  

    def speedup(self, old, name):
        if self.xaxis != old.xaxis:
            raise ValueError("x axis mismatch")
        data = {}
        for k, times in self.data.items():
            data[k] = []
            for i, time in enumerate(times):
                data[k].append(old.data[k][i] / time)
        return Measurement(None, self.xaxis, self.xlabel, data, name=name)

    def write_csv(self, filename):
        with open(filename, 'w') as csvfile:
            fieldnames = [self.xlabel] + list(self.data.keys())
            if self.sigma:
                fieldnames += ['sigma ' + k for k in self.data.keys()]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for i, x in enumerate(self.xaxis):
                row = {self.xlabel: x}
                for k in self.data.keys():
                    row[k] = self.data[k][i]
                if self.sigma:
                    for k in self.sigma.keys():
                        row['sigma ' + k] = self.sigma[k][i]
                writer.writerow(row)

    def write_settings(self, filename):
        with open(filename, 'w') as f:
            json.dump(self.instance.__dict__, f, indent=4)

    def plot(self, filename=None, **plot_args):
        for key, series in self.data.items():
            if self.sigma and self.sigma[key]:
                plt.errorbar(self.xaxis, series, self.sigma[key], label=key, **plot_args)
            else:
                plt.plot(self.xaxis, series, label=key, **plot_args)
        plt.xlabel(self.xlabel)
        plt.legend()
        if filename:
            plt.savefig(filename)

    def save(self):
        if self.instance:
            self.write_settings(self.name + '.json')
        self.write_csv(self.name + '.csv')
        self.plot(self.name + '.svg')
