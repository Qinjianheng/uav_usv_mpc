| Run | H | Active n | XY ref/track/actual RMSE m | Z ref/track/actual RMSE m |
|---|---:|---:|---|---|
| b_f2_1 | 1.2 | 447 | 2.6795/0.1006/2.6961 | 0.0773/0.0111/0.0796 |
| b_f2_2 | 1.2 | 485 | 0.9272/0.1071/0.9192 | 0.0528/0.0086/0.0519 |
| c_f2_1 | 1.6 | 515 | 0.6105/0.0768/0.5639 | 1.5163/0.0532/1.5079 |
| c_f2_2 | 1.6 | 457 | 0.3850/0.0510/0.3539 | 0.5227/0.0589/0.5212 |

Population: navigation state epochs 20–60 s after first FOLLOW diagnostic; active MINCO only, same-epoch truth brackets <=150 ms, no extrapolation. Each report also separates mixed-owner physical navigation samples and 0–20/60–end intervals. Reference RMSE and tracking RMSE cannot be subtracted: the reports include vector cross terms and centered cross covariance.

The H1.6 vertical error is already present in accepted MINCO references; the vehicle stays much closer to these references than to z=-5 m. These four logs establish that observation, not a horizon-only causal effect. b_f2_1 has substantial horizontal reference error.

Pre-shaping velocities and safety states were not retained by the recorder. derived_shaping_replay.csv is a source-matched reconstruction with output residuals; it is not direct observation. Native PX4 setpoints are decimated and remain at their own mapped publication epochs. Gray lines show observed controller-owner switches; association is not a flight counterfactual.
