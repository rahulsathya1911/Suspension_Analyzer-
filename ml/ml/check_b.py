import pandas as pd
from model import split_dataset
from envelope import fit_envelope, evaluate_envelope

df = pd.read_csv("../../out/comfort_dataset.csv")
train, cal, test = split_dataset(df)

env = fit_envelope(train)
res = evaluate_envelope(env, df)
res["split"] = df["split"].values

print("Flag rate by split (train/test should be near 0):")
print(res.groupby("split")["flagged"].mean())
print()
print(res[res.split == "ood"][["scenario_id", "flagged", "reason"]].to_string())
print()
print("y_ref - p_phys:")
print((df.y_ref - df.p_phys).describe())