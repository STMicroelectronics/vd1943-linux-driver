Repository Organization
-----------------------

- `.` folder contains source code + Makefile
- `dts` folder contains known to wort devicetree overlays

Branching Model
---------------

The proposed Branching Model is a simplified version of [GitFlow](https://datasift.github.io/gitflow/IntroducingGitFlow.html)

```
                   0.1         0.2                     0.X
                    |           |                       |
    *---------------*-----------*-----------------------* master
     \             /           /                       /
      *---*---*---*---*---*---*---*---*---*---*---*---* develop
           \         /                 \         /
            *---*---* feature-x         *---*---* feature-y

```

1. `master` and `develop`: the two main branches

    - `master`: the main branch, with "production-ready" content (hold the different release TAGS)
    - `develop`: contains the latest delivered development change (should be relatively stable)

2. `feature-x` branches

    - Branch off from `develop` and merge back to `develop`
    - Interesting when multiple developpers are contributing on different feature at the same time
    - Used to develop new features requiring significant amount of work (i.e. multiple commits)
    - Depending of the nature of the feature branch, someone may want to keep the historical existence of the branch (use of `--no-ff` argument while merging) or not (default merge behavior).

Release Reminder
----------------

1. Update Changelog (use git log to get overview of commits since last tag)

    `git log 0.6.0..HEAD --pretty=%s --reverse`

2. Tag 'master' (anotated tag)
3. Merge branch 'master' to 'debian'
4. Update debian changelog
