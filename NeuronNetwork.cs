using static MathNet.Numerics.Combinatorics;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using MathNet.Numerics;
using System.Linq;

namespace CsharpVersion
{
    public enum ActivationFunction
    {
        sigm,
        tanh_opt
    }

    public enum OutputFunction
    {
        sigm,
        softmax,
        linear,
        tanh_opt
    }

    public struct Options
    {
        public int BatchSize;
        public int NumberEpochs;
        public bool Validation;
        public bool DoPlot;
    }

    public class Loss
    {
        public List<double> train_e;
        public List<double> train_e_frac;
        public List<double> val_e;
        public List<double> val_e_frac;
        public Loss()
        {
            train_e = new List<double>();
            train_e_frac = new List<double>();
            val_e = new List<double>();
            val_e_frac = new List<double>();
        }
    }

    public class NeuronNetwork
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;
        int[] size;
        int layerNumber;

        #region Configuration
        //  Activation functions of hidden layers: 'sigm' (sigmoid) or 'tanh_opt' (optimal tanh).
        ActivationFunction activationFunction = ActivationFunction.tanh_opt;
        //  learning rate Note: typically needs to be lower when using 'sigm' activation function and non-normalized inputs.
        double learningRate = 0.002;
        //  Momentum
        double momentum = 0.9;
        //  Scaling factor for the learning rate(each epoch)
        double scaling_learningRate = 1;
        //  L2 regularization
        double weightPenaltyL2 = 0;
        //  Non sparsity penalty
        double nonSparsityPenalty = 0;
        //  Sparsity target
        double sparsityTarget = 0;
        //  Used for Denoising AutoEncoders
        double inputZeroMaskedFraction = 0;
        //  Dropout level(http://www.cs.toronto.edu/~hinton/absps/dropout.pdf)
        double dropoutFraction = 0;
        //  Internal variable.nntest sets this to one.
        double testing = 0;
        //  output unit 'sigm' (=logistic), 'softmax' and 'linear'
        OutputFunction output = OutputFunction.softmax;

        Options Option;
        #endregion

        public Matrix<double>[] W;
        Matrix<double>[] vW;
        Matrix<double>[] vWt;
        Vector<double>[] p;
        Matrix<double>[] dW;

        Matrix<double>[] a;
        Matrix<double> e;
        double L;
        Matrix<double>[] dropOutMask;
        Loss loss;

        public NeuronNetwork(int[] architecture, Options opts)
        {
            this.Option = opts;
            size = (int[])architecture.Clone();
            layerNumber = size.Length;
            W = new Matrix<double>[layerNumber - 1];
            vW = new Matrix<double>[layerNumber - 1];
            vWt = new Matrix<double>[layerNumber - 1];
            dW = new Matrix<double>[layerNumber - 1];
            p = new Vector<double>[layerNumber];
            p[0] = null;
            for (int i = 1; i < layerNumber; i++)
            {
                // weights and weight momentum
                W[i - 1] = (mb.Random(size[i], size[i - 1] + 1, new ContinuousUniform()) - 0.5)
                    * 2 * 4 * Math.Sqrt(6.0 / (size[i] + size[i - 1]));
                vW[i - 1] = mb.Dense(W[i - 1].RowCount, W[i - 1].ColumnCount, 0);
                vWt[i - 1] = mb.Dense(W[i - 1].RowCount, W[i - 1].ColumnCount, 0);
                //Console.WriteLine(W[i - 1]);
                // average activations(for use with sparsity)
                p[i] = vb.Dense(size[i]);
            }
            a = new Matrix<double>[layerNumber];
            //e = new matrix<double>[layernumber];
            L = 0;
            dropOutMask = new Matrix<double>[layerNumber];
            dropOutMask[0] = null;
            dropOutMask[layerNumber - 1] = null;
            loss = new Loss();
        }

        public void TrainNetwork(
            Matrix<double> train_x,
            Matrix<double> train_y,
            Matrix<double> val_x = null,
            Matrix<double> val_y = null)
        {
            // NNTRAIN trains a neural net
            // [nn, L] = nnff(nn, x, y, opts) trains the neural network nn with input x and
            // output y for opts.numepochs epochs, with minibatches of size
            // opts.batchsize.Returns a neural network nn with updated activations,
            // errors, weights and biases, (nn.a, nn.e, nn.W, nn.b) and L, the sum
            // squared error for each training minibatch.

            //assert(isfloat(train_x), 'train_x must be a float');
            //assert(nargin == 4 || nargin == 6, 'number ofinput arguments must be 4 or 6')


            Option.Validation = false;

            if (val_x != null && val_y != null)
            {
                Option.Validation = true;
            }
            //fhandle = [];
            //if isfield(opts, 'plot') && opts.plot == 1
            //    fhandle = figure();
            //end

            int m = train_x.RowCount;
            int batchsize = Option.BatchSize;
            int numepochs = Option.NumberEpochs;

            int numbatches = m / batchsize;

            //assert(rem(numbatches, 1) == 0, 'numbatches must be a integer');

            List<double> L = new(Option.NumberEpochs * numbatches);
            int n = 0;
            for (int i = 0; i < Option.NumberEpochs; i++)
            {
                //Vector<int> kk = Vector<int>.Build.Dense();
                Permutation permutation = new(GeneratePermutation(m));
                var tempX = train_x.Clone();
                var tempY = train_y.Clone();
                tempX.PermuteRows(permutation);
                tempY.PermuteRows(permutation);
                for (int l = 1; l < numbatches; l++)
                {
                    var batch_x = tempX.SubMatrix((l - 1) * batchsize, batchsize, 0, tempX.ColumnCount);
                    // Add noise to input(for use in denoising autoencoder)
                    if (inputZeroMaskedFraction != 0)
                    {
                        var mask = mb.Random(batch_x.RowCount, batch_x.ColumnCount, new ContinuousUniform())
                            .Map<double>(s => s > inputZeroMaskedFraction ? 1 : 0);
                        batch_x = batch_x.PointwiseMultiply(mask);
                    }
                    var batch_y = tempY.SubMatrix((l - 1) * batchsize, batchsize, 0, tempY.ColumnCount);
                    //Console.WriteLine(batch_x.Append(batch_y).ToString());
                    FeedForward(batch_x, batch_y);
                    BackPropagate();
                    ApplyGradients();

                    L.Add(this.L);
                    n++;
                }
                // plot(L);
                if (Option.Validation)
                {
                    Evaluate(train_x, train_y, val_x, val_y);
                    //str_perf = sprintf('; Full-batch train mse = //f, val mse = //f', loss.train.e(end), loss.val.e(end));
                }
                else
                {
                    Evaluate(train_x, train_y);
                    //str_perf = sprintf('; Full-batch train err = //f', loss.train.e(end));
                }
                //if ishandle(fhandle)
                //    nnupdatefigures(nn, fhandle, loss, opts, i);
                //disp(['epoch ' num2str(i) '/' num2str(opts.numepochs) '. Took ' num2str(t) ' seconds' '. Mini-batch mean squared error on training set is ' num2str(mean(L((n - numbatches):(n - 1)))) str_perf]);                //if ishandle(fhandle)
                //    nnupdatefigures(nn, fhandle, loss, opts, i);
                //disp(['epoch ' num2str(i) '/' num2str(opts.numepochs) '. Took ' num2str(t) ' seconds' '. Mini-batch mean squared error on training set is ' num2str(mean(L((n - numbatches):(n - 1)))) str_perf]);
                learningRate *= scaling_learningRate;
            }
            //foreach (var item in L)
            //{
            //    Console.WriteLine(item);
            //}
            //Console.WriteLine();
        }

        public void FeedForward(Matrix<double> x, Matrix<double> y)
        {
            // number of train sample
            var m = x.RowCount;

            x = mb.Dense(m, 1, 1).Append(x);
            a[0] = x;

            // feedforward pass
            for (int i = 1; i < layerNumber - 1; i++)
            {
                switch (activationFunction)
                {
                    case ActivationFunction.sigm:
                        // Calculate the unit's outputs (including the bias term)
                        a[i] = Sigmoid(a[i - 1] * W[i - 1].Transpose());
                        break;
                    case ActivationFunction.tanh_opt:
                        a[i] = TanhOptimized(a[i - 1] * W[i - 1].Transpose());
                        break;
                }

                // dropout
                if (dropoutFraction > 0)
                {
                    if (testing > 0)
                    {
                        a[i] = a[i] * (1 - dropoutFraction);
                    }
                    else
                    {
                        dropOutMask[i] = mb.Random(a[i].RowCount, a[i].ColumnCount, new ContinuousUniform())
                            .Map<double>(s => s > dropoutFraction ? 1 : 0);
                        a[i] = a[i].PointwiseMultiply(dropOutMask[i]);
                    }
                }

                // calculate running exponential activations for use with sparsity
                if (nonSparsityPenalty > 0)
                {
                    p[i] = 0.99 * p[i] + 0.01 * a[i].ColumnSums() / a[i].RowCount;
                }
                // Add the bias term
                a[i] = mb.Dense(m, 1, 1).Append(a[i]);
            }
            switch (output)
            {
                case OutputFunction.tanh_opt:
                    a[layerNumber - 1] = TanhOptimized(a[layerNumber - 2] * W[layerNumber - 2].Transpose());
                    break;
                case OutputFunction.sigm:
                    a[layerNumber - 1] = Sigmoid(a[layerNumber - 2] * W[layerNumber - 2].Transpose());
                    break;
                case OutputFunction.linear:
                    a[layerNumber - 1] = a[layerNumber - 2] * W[layerNumber - 2].Transpose();
                    break;
                case OutputFunction.softmax:
                    a[layerNumber - 1] = Softmax(a[layerNumber - 2] * W[layerNumber - 2].Transpose());
                    break;
            }

            // error and loss
            e = y - a[^1];

            L = output switch
            {
                OutputFunction.linear or OutputFunction.sigm or OutputFunction.tanh_opt =>
                    1.0 / 2 * e.PointwisePower(2).RowSums().Sum() / m,
                OutputFunction.softmax =>
                    -y.PointwiseMultiply(a[layerNumber - 1].PointwiseLog()).RowSums().Sum() / m,
                _ => 0,
            };
        }

        public void BackPropagate()
        {
            int n = layerNumber;
            Matrix<double> sparsityError = null;
            Matrix<double>[] d = new Matrix<double>[n];
            d[0] = null;
            d[n - 1] = output switch
            {
                OutputFunction.tanh_opt =>
                    -1.0 / 2 * e.PointwiseMultiply(1 - a[n - 1].PointwisePower(2)),
                OutputFunction.sigm =>
                    -e.PointwiseMultiply(a[n - 1].PointwiseMultiply(1 - a[n - 1])),
                OutputFunction.softmax or OutputFunction.linear =>
                    -e,
                _ => null,
            };

            for (int i = n - 2; i > 0; i--)
            {
                Matrix<double> d_act = activationFunction switch
                {
                    ActivationFunction.sigm => a[i].PointwiseMultiply(1 - a[i]),
                    ActivationFunction.tanh_opt => 1.0 / 2 * (1 - a[i].PointwisePower(2)),
                    _ => null,
                };

                if (nonSparsityPenalty > 0)
                {
                    Matrix<double> pi = mb.DenseOfRowVectors(
                        Enumerable.Repeat(p[i], a[i].RowCount));
                    sparsityError = mb.Dense(a[i].RowCount, 1, 0).Append(
                        nonSparsityPenalty * (sparsityTarget / pi + (1 - sparsityTarget) / (1 - pi))
                    );
                }

                sparsityError ??= mb.Dense(a[i].RowCount, p[i].Count + 1, 0);
                // Backpropagate first derivatives
                if (i + 1 == n - 1)
                {
                    // in this case in d{n} there is not the bias term to be removed
                    d[i] = (d[i + 1] * W[i] + sparsityError)
                        .PointwiseMultiply(d_act); // Bishop(5.56)
                }
                else
                { // in this case in d{i} the bias term has to be removed
                    d[i] = (d[i + 1].SubMatrix(0, d[i + 1].RowCount, 1, d[1 + 1].ColumnCount - 1)
                        * W[i] + sparsityError)
                        .PointwiseMultiply(d_act);
                }

                if (dropoutFraction > 0)
                {
                    d[i] = d[i].PointwiseMultiply(
                        mb.Dense(d[i].RowCount, 1, 1).Append(dropOutMask[i])
                    );
                }
            }

            for (int i = 0; i < (n - 1); i++)
            {
                if (i + 1 == n - 1)
                {
                    dW[i] = (d[i + 1].Transpose() * a[i]) / d[i + 1].RowCount;
                }
                else
                {
                    dW[i] = (d[i + 1].SubMatrix(
                            0, d[i + 1].RowCount, 1, d[i + 1].ColumnCount - 1)
                        .Transpose() * a[i]) / d[i + 1].RowCount;
                }
            }
        }

        public void ApplyGradients()
        {
            // NNAPPLYGRADS updates weights and biases with calculated gradients
            // nn = nnapplygrads(nn) returns an neural network structure with updated
            // weights and biases

            Matrix<double> dWtemp;
            for (int i = 0; i < (layerNumber - 1); i++)
            {
                if (weightPenaltyL2 > 0)
                {
                    dWtemp = dW[i]
                    + weightPenaltyL2 * mb.Dense(W[i].RowCount, 1, 0).Append(W[i].SubMatrix(
                            0, W[i].RowCount, 1, W[i].ColumnCount - 1));
                }
                else
                {
                    dWtemp = dW[i];
                }

                // dW = nn.learningRate * dW;

                if (momentum > 0)
                {
                    // nn.vW{ i} = nn.momentum * nn.vW{ i} +dW;
                    // dW = nn.vW{ i};

                    vW[i] = dWtemp;
                    vWt[i] = momentum * vWt[i] + (1 - momentum) * dWtemp.PointwisePower(2);
                }
                // nn.W{ i} = nn.W{ i} - dW;
                //Console.WriteLine(W[i]);
                W[i] = W[i] - learningRate * vW[i].PointwiseDivide((vWt[i] + 1e-8).PointwiseSqrt());
                //Console.WriteLine(W[i]);
            }
        }

        public void Evaluate(
            Matrix<double> train_x,
            Matrix<double> train_y,
            Matrix<double> val_x = null,
            Matrix<double> val_y = null)
        {
            // NNEVAL evaluates performance of neural network
            // Returns a updated loss struct
            //assert(nargin == 4 || nargin == 6, 'Wrong number of arguments');

            testing = 1;
            // training performance
            FeedForward(train_x, train_y);
            loss.train_e.Add(L);

            // validation performance
            if (val_x != null && val_y != null)
            {
                FeedForward(val_x, val_y);
                loss.val_e.Add(L);
            }

            testing = 0;
            //calc misclassification rate if softmax
            if (output == OutputFunction.softmax)
            {
                var er_train = Test(train_x, train_y);
                loss.train_e_frac.Add(er_train);
            }

            if (val_x != null && val_y != null)
            {
                var er_val = Test(val_x, val_y);
                loss.val_e_frac.Add(er_val);
            }
        }

        public double Test(Matrix<double> x, Matrix<double> y)
        {
            var labels = Predict(x);
            var expected = vb.DenseOfEnumerable(from item in y.EnumerateRows()
                                                select (double)item.MaximumIndex());

            labels.Map2((lab, exp) => lab == exp ? 0 : 1, expected);
            //bad = find(labels ~= expected);
            return labels.Sum() / x.RowCount;
        }

        public Vector<double> Predict(Matrix<double> x)
        {
            testing = 1;
            FeedForward(x, mb.Dense(x.RowCount, size.Last()));
            testing = 0;
            var t = a.Last().EnumerateRows();
            //Console.WriteLine(a.Last().ToString(a.Last().RowCount, a.Last().ColumnCount));
            var q = from item in t
                    select (double)item.MaximumIndex();
            return vb.DenseOfEnumerable(q);
        }

        #region Helper Function
        static Matrix<double> TanhOptimized(Matrix<double> matrix)
        {
            //return matrix.Map(s => 1.7159 * Trig.Tanh(2.0 / 3 * s));
            return matrix.Map(s => Trig.Tanh(s / 2));
        }

        static Matrix<double> Sigmoid(Matrix<double> matrix)
        {
            return matrix.Map(SpecialFunctions.Logistic);
        }

        static Matrix<double> Softmax(Matrix<double> matrix)
        {
            // softmax(x[i]) = exp(x[i]) / sum(exp(x[j]))
            var matrixa =
               (matrix - Matrix<double>.Build.DenseOfColumnVectors(
                       Enumerable.Repeat(
                           // 获取每一行的最大值， 可能是为了避免浮点数精度表示的问题
                           matrix.ReduceColumns((prev, curr) =>
                           {
                               return prev.Map2((pr, cu) => pr > cu ? pr : cu, curr);
                           }),
                   matrix.ColumnCount)))
               .PointwiseExp();
            var matrixb = matrixa.PointwiseDivide(
                Matrix<double>.Build.DenseOfColumnVectors(
                    Enumerable.Repeat(matrixa.RowSums(), matrix.ColumnCount)));
            return matrixb;
        }

        #endregion
    }
}
