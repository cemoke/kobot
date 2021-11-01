#!/bin/zsh
dotfiles_dir=$HOME/kobot_ws/src/kobot/dotfiles
echo "Dotfiles directory is $dotfiles_dir"
ln -sf $dotfiles_dir/.zshrc $HOME/.zshrc
ln -sf $dotfiles_dir/.tmux.conf $HOME/.tmux.conf
for f in *.yml
do echo "Processing $f file.."
  ln -sf $dotfiles_dir/$f $HOME/.tmuxinator/$f
done
