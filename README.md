To edit the actual report edit `/content/dla/_index.md` and write what you
want to write BELOW all the plusses at the top.

A push to main should generate changes that can be seen [here](https://nathanielnrn.github.io/hough-transform-documentation/)

Things should be written in [markdown](https://commonmark.org/help/) with a few caveats

## Images

Images should be inserted using

`{{ figure(src="img.png", alt="alt text", caption="caption text") }}`
with the file located in the `content/dla/` directory.


Rendering of math should work by just using $ and $$


We can add webms in particular using
`{{ webm(src="basic.webm", caption="Figure 1: Basic DLA (non-cyclic)", width=500) }}`

Links are done by `[display text](url)`

Headers are done with `###` (More `#` makes things smaller) 
