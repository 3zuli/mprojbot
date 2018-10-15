# mprojbot
MPROJ robot project files


## Development

### Version control with git

Recommendation: Use GitKraken, it's free for personal use.

Main principles:
* master branch is (mostly) stable
* development is done on "feature" (or "topic") branches
* ideal rule: one "feature" = one commit
* avoid merge commits, rebase before merge

#### Feature branch
A "feature" or "topic" branch should concentrate on solving one specific task.
It can be a bug fix, a new feature, or new documentation. This should help
to better split the work for multiple developers and to create less merge
conflicts when integrating the work into the master.

Lifecycle:

1. create a feature branch from the actual master
```sh
git checkout master
git pull origin master
git checkout -b <feature_branch>
```

2. continously commit related changes, rebase to the actual master
```sh
git add ...
git commit -m ...
git pull --rebase origin master
```

3. push the branch to github and create a pull request
```sh
git push --set-upstream origin <feature_branch>
```

4. merge to master
```sh
git checkout master
git merge <feature_branch>
git push
```
